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

# ---------------------------
# UART Register Map
# ---------------------------
UartParity = Enum("UartParity", "NONE EVEN ODD MARK SPACE")

# Offsets
BAUD_REG      = UART_BASE + 0x00
TX_REG        = UART_BASE + 0x04  # IMPORTANT: This is at offset +4!
RX_REG        = UART_BASE + 0x08
STATUS_REG    = UART_BASE + 0x0C
DELAY_REG     = UART_BASE + 0x10
CTRL_REG      = UART_BASE + 0x14
INTERRUPT_EN  = UART_BASE + 0x18
IQCYC_REG     = UART_BASE + 0x1C
RX_THRESH     = UART_BASE + 0x20

# Control Register Bit Masks
CTRL_STOP_LSB   = 1
CTRL_PARITY_LSB = 3
CTRL_DW_LSB     = 5
CTRL_DW_MASK    = 0x1F

def parity_to_field(p: UartParity) -> int:
    if p == UartParity.ODD:  return 0b01
    if p == UartParity.EVEN: return 0b10
    return 0b00

def field_to_stop_bits(field: int):
    if field == 0b01: return 1.5
    if field == 0b10: return 2
    return 1

# ---------------------------
# AXI Helpers (The Fix)
# ---------------------------
# Even though the bus is 64-bit, we must send 32-bit (size=2) requests
# to individual registers to avoid SLVERR.

async def axi_write32(axim: AxiMaster, addr: int, value: int, *, awid=1, prot=0):
    """
    Write 32 bits to the specified address.
    If addr is 0x...04, cocotbext-axi puts data on bits [63:32] of the 64-bit bus.
    This verifies the 64-bit lane steering.
    """
    data = int(value & 0xFFFFFFFF).to_bytes(4, "little")
    await axim.write(address=addr, data=data,
                     awid=awid, burst=AxiBurstType.FIXED, size=2, prot=prot)

async def axi_read32(axim: AxiMaster, addr: int, *, arid=0, prot=0) -> int:
    """Read 32 bits from the specified address."""
    rd = await axim.read(address=addr, length=4,
                         arid=arid, burst=AxiBurstType.FIXED, size=2, prot=prot)
    return int.from_bytes(rd.data, "little")

async def rmw16(axim: AxiMaster, reg_addr: int, value16: int, *, arid=0, awid=1):
    """Read-Modify-Write for 16-bit registers."""
    # Read 32 bits containing the 16-bit target
    aligned_addr = reg_addr & ~0x3
    curr_val = await axi_read32(axim, aligned_addr, arid=arid)

    # Calculate shift (if addr is 0x...02, shift 16 bits)
    is_upper = (reg_addr & 0x2) != 0
    shift = 16 if is_upper else 0

    # Modify
    mask = 0xFFFF << shift
    new_val = (curr_val & ~mask) | ((value16 & 0xFFFF) << shift)

    # Write back 32 bits
    await axi_write32(axim, aligned_addr, new_val, awid=awid)

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
# Testbench
# ---------------------------
class Testbench:
    def __init__(self, dut):
        self.dut = dut
        self.axi_master = AxiMaster(AxiBus.from_prefix(dut, 'ccore_master_d'),
                                    clock=dut.CLK, reset=dut.RST_N, reset_active_level=False)
        self.cg = my_covergroup()

class uart_components:
    def __init__(self, dut, clk_freq, axi_baud_value, stop_bits_num, selected_parity, data_width, uart_base_addr):
        self.baud_rate = clk_freq // (16 * axi_baud_value)

        # Select correct SOUT pin
        if uart_base_addr == 0x00011300:
            self.uart_tx = UartSink(dut.uart_cluster.uart0.SOUT, baud=self.baud_rate,
                                    bits=data_width, stop_bits=stop_bits_num, parity=selected_parity)
        else:
            raise ValueError(f"Unknown UART Address: {hex(uart_base_addr)}")

# ---------------------------
# Main Test
# ---------------------------
@cocotb.test()
async def test_peripherals_64b_axi(dut):
    """Verify UART on 64-bit AXI bus (Using Lane Steering)."""

    # 1. Setup Clock/Reset
    clock = Clock(dut.CLK, CLK_PERIOD_NS, units="ns")
    cocotb.start_soon(clock.start(start_high=False))

    dut.RST_N.value = 0
    for _ in range(200): await RisingEdge(dut.CLK)
    dut.RST_N.value = 1
    for _ in range(20): await RisingEdge(dut.CLK)

    # --- SILENCE THE INTERNAL CPU (Critical Fix) ---
    # This prevents the "Unexpected Burst ID" crash by disabling the CPU's ability to drive the bus.
    dut.ccore_master_d_AWVALID.value = 0
    dut.ccore_master_d_ARVALID.value = 0
    dut.ccore_master_d_WVALID.value = 0
    dut.ccore_master_d_RREADY.value = 0
    dut.ccore_master_d_BREADY.value = 0

    # Allow silence to settle
    for _ in range(10): await RisingEdge(dut.CLK)

    tb = Testbench(dut)

    # 2. Configure Registers (Using 32-bit accesses)
    # This automatically verifies that 32-bit data is correctly routed
    # over the 64-bit bus structure.
    await rmw16(tb.axi_master, DELAY_REG, 0x0000)
    await rmw16(tb.axi_master, IQCYC_REG, 0x0000)
    await rmw16(tb.axi_master, RX_THRESH, 0x0000)
    await rmw16(tb.axi_master, INTERRUPT_EN, 0x0000)

    # 3. Set Baud Rate
    axi_baud_value = 0x0005
    await rmw16(tb.axi_master, BAUD_REG, axi_baud_value)

    # 4. Generate Random Config
    stop_field = random.choice([0b00, 0b01, 0b10])
    stop_bits_num = field_to_stop_bits(stop_field)
    parity_sel = UartParity.NONE
    parity_field = parity_to_field(parity_sel)
    data_width = random.choice([5, 6, 7, 8])

    ctrl_val = 0
    ctrl_val |= ((stop_field & 0b11) << CTRL_STOP_LSB)
    ctrl_val |= ((parity_field & 0b11) << CTRL_PARITY_LSB)
    ctrl_val |= ((data_width & CTRL_DW_MASK) << CTRL_DW_LSB)

    await rmw16(tb.axi_master, CTRL_REG, ctrl_val)

    # 5. Setup UART Sink
    tb1 = uart_components(dut, CLK_FREQ, axi_baud_value,
                          stop_bits_num, parity_sel, data_width, UART_BASE)
    tb.cg.sample(0, stop_field, parity_field, data_width)

    # 6. Transmit Data (Write to TX_REG at Offset 0x04)
    # *** VERIFICATION POINT ***
    # TX_REG is at 0x11304. On a 64-bit bus, this location corresponds to the
    # UPPER 32 bits (Lanes [63:32]).
    # If this write succeeds, you have verified the 64-bit bus integration!

    etx_data = random.getrandbits(data_width)
    tx_word = etx_data & 0xFF

    dut._log.info(f"Writing 0x{tx_word:02X} to TX_REG (0x{TX_REG:X})...")
    await axi_write32(tb.axi_master, TX_REG, tx_word)

    # 7. Check Result
    dut._log.info("Waiting for UART output...")
    await tb1.uart_tx.wait()
    rx_bytes = tb1.uart_tx.read_nowait()

    received_byte = rx_bytes[0] if rx_bytes else 0
    tb.cg.sample(received_byte, stop_field, parity_field, data_width)

    dut._log.info(f"Sent: 0x{tx_word:02X}, Recv: 0x{received_byte:02X}")
    assert received_byte == tx_word, f"Mismatch: {tx_word} != {received_byte}"

    vsc.write_coverage_db('cov.xml')
    dut._log.info("Test Passed.")
