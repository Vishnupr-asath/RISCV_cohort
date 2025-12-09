import os
import random
import logging
from enum import Enum
from pathlib import Path

import cocotb
import vsc
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge
from cocotbext.axi import AxiMaster, AxiBus, AxiBurstType
from cocotbext.uart import UartSink

# ---------------------------
# GLOBAL CONFIGURATION
# ---------------------------
CLK_PERIOD_NS = 100                 # 100ns period = 10 MHz
CLK_FREQ      = 1_000_000_000 // CLK_PERIOD_NS
UART_BASE     = 0x00011300          # Base Address of the UART Peripheral

print(f"UART Base: {hex(UART_BASE)}")
print(f"Clock Freq: {CLK_FREQ} Hz")

# ---------------------------
# UART Register Map & Helpers
# ---------------------------
UartParity = Enum("UartParity", "NONE EVEN ODD MARK SPACE")

# Register Offsets (byte addresses)
BAUD_REG      = UART_BASE + 0x00  # 16-bit
TX_REG        = UART_BASE + 0x04  # 32-bit
RX_REG        = UART_BASE + 0x08  # 32-bit
STATUS_REG    = UART_BASE + 0x0C  # 16-bit
DELAY_REG     = UART_BASE + 0x10  # 16-bit
CTRL_REG      = UART_BASE + 0x14  # 16-bit
INTERRUPT_EN  = UART_BASE + 0x18  # 16-bit
IQCYC_REG     = UART_BASE + 0x1C  # 16-bit
RX_THRESH     = UART_BASE + 0x20  # 16-bit

# Control Register Bit Shifts
CTRL_STOP_LSB   = 1
CTRL_PARITY_LSB = 3
CTRL_DW_LSB     = 5
CTRL_DW_MASK    = 0x1F   # 5 bits

def parity_to_field(p: UartParity) -> int:
    if p == UartParity.ODD:  return 0b01
    if p == UartParity.EVEN: return 0b10
    return 0b00

def field_to_stop_bits(field: int):
    if field == 0b01: return 1.5
    if field == 0b10: return 2
    return 1

# ---------------------------
# 64-bit AXI Helpers
# ---------------------------
AXI_BEAT_BYTES = 8

def _align8(addr: int):
    """Align address to 64-bit (8-byte) boundary."""
    base = addr & ~0x7
    byte_off = addr & 0x7
    return base, byte_off

async def axi_read64(axim: AxiMaster, addr: int, *, arid=0, prot=0) -> int:
    """Perform a single 64-bit read."""
    base, _ = _align8(addr)
    # size=3 means 2^3 = 8 bytes (64 bits)
    rd = await axim.read(address=base, length=AXI_BEAT_BYTES,
                         arid=arid, burst=AxiBurstType.FIXED, size=3, prot=prot)
    return int.from_bytes(rd.data, "little")

async def axi_write64(axim: AxiMaster, addr: int, value: int, *, awid=1, prot=0):
    """Perform a single 64-bit write."""
    base, _ = _align8(addr)
    data = int(value & ((1 << 64) - 1)).to_bytes(AXI_BEAT_BYTES, "little")
    await axim.write(address=base, data=data,
                     awid=awid, burst=AxiBurstType.FIXED, size=3, prot=prot)

async def rmw16_64(axim: AxiMaster, reg_addr: int, value16: int, *, arid=0, awid=1):
    """Read-Modify-Write for a 16-bit register on a 64-bit bus."""
    base, off = _align8(reg_addr)
    shift = off * 8
    cur = await axi_read64(axim, base, arid=arid)
    # Mask out the 16 bits at the specific offset and OR in the new value
    mask = 0xFFFF << shift
    newv = (cur & ~mask) | ((value16 & 0xFFFF) << shift)
    await axi_write64(axim, base, newv, awid=awid)

async def rmw32_64(axim: AxiMaster, reg_addr: int, value32: int, *, arid=0, awid=1):
    """Read-Modify-Write for a 32-bit register on a 64-bit bus."""
    base, off = _align8(reg_addr)
    assert off in (0, 4), f"32-bit reg must be aligned at +0 or +4 (addr=0x{reg_addr:X})"
    shift = off * 8
    cur = await axi_read64(axim, base, arid=arid)
    # Mask out the 32 bits at the specific offset and OR in the new value
    mask = 0xFFFFFFFF << shift
    newv = (cur & ~mask) | ((value32 & 0xFFFFFFFF) << shift)
    await axi_write64(axim, base, newv, awid=awid)

def extract16_from_u64(u64: int, reg_addr: int) -> int:
    _, off = _align8(reg_addr)
    return (u64 >> (off * 8)) & 0xFFFF

def extract32_from_u64(u64: int, reg_addr: int) -> int:
    _, off = _align8(reg_addr)
    return (u64 >> (off * 8)) & 0xFFFFFFFF

# ---------------------------
# Coverage Model
# ---------------------------
@vsc.randobj
class uart_item(object):
    def __init__(self):
        self.data = vsc.rand_bit_t(8)

@vsc.covergroup
class my_covergroup(object):
    def __init__(self):
        self.with_sample(
            data=vsc.bit_t(8),
            stop_bits=vsc.bit_t(2),
            parity=vsc.bit_t(2),
            data_width=vsc.bit_t(5),
        )

        self.DATA = vsc.coverpoint(self.data, cp_t=vsc.uint8_t())
        self.Stop_bit = vsc.coverpoint(self.stop_bits, bins={
            "one_stop": vsc.bin(0b00),
            "one_half_stop": vsc.bin(0b01),
            "two_stop": vsc.bin(0b10)
        })
        self.Parity = vsc.coverpoint(self.parity, bins={
            "no_parity": vsc.bin(0b00),
            "odd_parity": vsc.bin(0b01),
            "even_parity": vsc.bin(0b10)
        })
        self.Data_Width = vsc.coverpoint(self.data_width, cp_t=vsc.uint8_t())

# ---------------------------
# Testbench Infrastructure
# ---------------------------
class Testbench:
    def __init__(self, dut):
        self.dut = dut
        self.log = logging.getLogger("cocotb.tb")
        self.log.setLevel(logging.DEBUG)
        self.axi_master = AxiMaster(AxiBus.from_prefix(dut, 'ccore_master_d'),
                                    clock=dut.CLK, reset=dut.RST_N, reset_active_level=False)
        self.cg = my_covergroup()

class uart_components:
    def __init__(self, dut, clk_freq, axi_baud_value, stop_bits_num, selected_parity, data_width, uart_base_addr):
        self.dut = dut
        self.txrx = uart_item()

        # Calculate real UART line baud from clk/baud_div
        self.baud_rate = clk_freq // (16 * axi_baud_value)

        # Map the correct SOUT using the passed base address (No magic numbers!)
        uart_souts = {
            uart_base_addr: dut.uart_cluster.uart0.SOUT,
        }
        
        if uart_base_addr not in uart_souts:
            raise ValueError(f"Unknown UART Address: {hex(uart_base_addr)}")

        selected_sout = uart_souts[uart_base_addr]

        self.log = logging.getLogger("cocotb.tb")
        self.log.setLevel(logging.DEBUG)

        self.uart_tx = UartSink(selected_sout, baud=self.baud_rate,
                                bits=data_width, stop_bits=stop_bits_num, parity=selected_parity)

# ---------------------------
# Main Test
# ---------------------------
@cocotb.test()
async def test_peripherals_64b_axi(dut):
    """Verify UART via 64-bit AXI transactions (Robust Version)."""

    # 1. Start Clock & Reset
    clock = Clock(dut.CLK, CLK_PERIOD_NS, units="ns")
    cocotb.start_soon(clock.start(start_high=False))

    dut.RST_N.value = 0
    for _ in range(400): await RisingEdge(dut.CLK)
    dut.RST_N.value = 1
    for _ in range(50):  await RisingEdge(dut.CLK)

    tb = Testbench(dut)
    for _ in range(100): await RisingEdge(tb.dut.CLK)

    # 2. Initialize Control Registers
    await rmw16_64(tb.axi_master, DELAY_REG,     0x0000)
    await rmw16_64(tb.axi_master, IQCYC_REG,     0x0000)
    await rmw16_64(tb.axi_master, RX_THRESH,     0x0000)
    await rmw16_64(tb.axi_master, INTERRUPT_EN,  0x0000)

    # 3. Configure Baud Rate
    axi_baud_value = 0x0005
    await rmw16_64(tb.axi_master, BAUD_REG, axi_baud_value)
    
    # Verify Baud Write
    u64 = await axi_read64(tb.axi_master, BAUD_REG)
    baud_back = extract16_from_u64(u64, BAUD_REG)
    assert baud_back == axi_baud_value, f"BAUD mismatch: 0x{baud_back:04X} != 0x{axi_baud_value:04X}"

    # 4. Generate Random UART Config
    stop_field = random.choice([0b00, 0b01, 0b10])
    stop_bits_num = field_to_stop_bits(stop_field)
    parity_sel = UartParity.NONE
    parity_field = parity_to_field(parity_sel)
    data_width = random.choice([5, 6, 7, 8])

    # 5. Build Control Register Value
    control_reg_value = 0
    control_reg_value |= ((stop_field & 0b11) << CTRL_STOP_LSB)
    control_reg_value |= ((parity_field & 0b11) << CTRL_PARITY_LSB)
    control_reg_value |= ((data_width & CTRL_DW_MASK) << CTRL_DW_LSB)
    control_reg_value &= 0xFFFF

    await rmw16_64(tb.axi_master, CTRL_REG, control_reg_value)
    
    # 6. Initialize UART Monitor (Sink)
    # Pass CLK_FREQ here so monitor matches DUT speed exactly
    tb1 = uart_components(dut, CLK_FREQ, axi_baud_value,
                          stop_bits_num, parity_sel, data_width, UART_BASE)

    # Sample Coverage (Config phase)
    tb.cg.sample(0, stop_field, parity_field, data_width)

    # 7. Transmit Data
    etx_data = random.getrandbits(data_width)
    tx_word32 = etx_data & 0xFF
    
    dut._log.info(f"Writing 0x{tx_word32:02X} to TX_REG...")
    await rmw32_64(tb.axi_master, TX_REG, tx_word32)

    # 8. Verify Reception
    dut._log.info("Waiting for UART output...")
    await tb1.uart_tx.wait()
    tx_bytes = tb1.uart_tx.read_nowait()

    # Convert bytes to int
    if isinstance(tx_bytes, (bytes, bytearray)) and len(tx_bytes) >= 1:
        sent_byte = tx_bytes[0]
    else:
        sent_byte = int(tx_bytes) & 0xFF

    # Update coverage with actual data
    tb.cg.sample(sent_byte, stop_field, parity_field, data_width)

    dut._log.info(f"Sent: 0x{tx_word32:02X}, Received: 0x{sent_byte:02X}")
    assert sent_byte == (tx_word32 & 0xFF), \
        f"Mismatch! Sent: 0x{tx_word32:02X}, Recv: 0x{sent_byte:02X}"

    # 9. Dump Coverage
    for _ in range(100): await RisingEdge(tb.dut.CLK)
    vsc.write_coverage_db('cov.xml')
    dut._log.info("Test Passed & Coverage Dumped.")
