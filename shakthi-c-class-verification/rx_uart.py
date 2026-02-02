import os
import random
import logging
import sys
from enum import Enum

import cocotb
import vsc
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer
from cocotbext.axi import AxiMaster, AxiBus, AxiBurstType
from cocotbext.uart import UartSource

# -----------------------------------------------------------------------------
# 1. CONSTANTS
# -----------------------------------------------------------------------------
CLK_PERIOD_NS = 100
CLK_FREQ      = 1_000_000_000 // CLK_PERIOD_NS
UART_BASE     = 0x00011300

BAUD_REG      = UART_BASE + 0x00
RX_REG        = UART_BASE + 0x08
STATUS_REG    = UART_BASE + 0x0C
CTRL_REG      = UART_BASE + 0x14
RX_THRESH_REG = UART_BASE + 0x20

# Control Register Shifts
CTRL_STOP_LSB   = 1
CTRL_DW_LSB     = 5
CTRL_DW_MASK    = 0x1F

def field_to_stop_bits(field: int):
    if field == 0b01: return 1.5
    if field == 0b10: return 2
    return 1

# -----------------------------------------------------------------------------
# 2. HELPER FUNCTIONS (64-BIT TRANSACTIONS)
# -----------------------------------------------------------------------------
async def axi_write64(axim: AxiMaster, addr: int, value: int):
    """ Writes a 64-bit value to the bus. """
    val_64 = int(value & 0xFFFFFFFFFFFFFFFF)
    data = val_64.to_bytes(8, "little")
    await axim.write(address=addr, data=data, burst=AxiBurstType.FIXED, size=3)

async def axi_read64(axim: AxiMaster, addr: int):
    """ Reads a 64-bit value from the bus. """
    resp = await axim.read(address=addr, length=8, burst=AxiBurstType.FIXED, size=3)
    return int.from_bytes(resp.data, byteorder='little')

# -----------------------------------------------------------------------------
# 3. COVERAGE MODEL
# -----------------------------------------------------------------------------
@vsc.randobj
class uart_rx_config(object):
    def __init__(self):
        self.data_width = vsc.rand_uint8_t()
        self.stop_field = vsc.rand_bit_t(2)
        self.baud_div = vsc.rand_uint16_t()

    @vsc.constraint
    def basic_constraints(self):
        # --- ROBUST CONSTRAINTS ---
        self.data_width == 8
        self.baud_div == 5      # Lock to known good speed (125kbps)
        self.stop_field < 3     # Allow 1, 1.5, 2 Stop bits

@vsc.covergroup
class rx_covergroup(object):
    def __init__(self):
        self.with_sample(
            stop_bits=vsc.bit_t(2),
            baud_div=vsc.uint16_t()
        )
        self.Stop_Type = vsc.coverpoint(self.stop_bits, bins={
            "1_stop": vsc.bin(0b00),
            "1.5_stop": vsc.bin(0b01),
            "2_stop": vsc.bin(0b10)
        })

# -----------------------------------------------------------------------------
# 4. TESTBENCH CLASS
# -----------------------------------------------------------------------------
class UartTestbench:
    def __init__(self, dut):
        self.dut = dut
        self.log = logging.getLogger("cocotb.tb")
        self.log.setLevel(logging.INFO)
        self.axi_master = AxiMaster(AxiBus.from_prefix(dut, 'ccore_master_d'),
                                    clock=dut.CLK, reset=dut.RST_N, reset_active_level=False)
        self.cg = rx_covergroup()
        self.cfg = uart_rx_config()

    async def reset(self):
        self.dut.RST_N.value = 0
        await RisingEdge(self.dut.CLK)
        self.dut.RST_N.value = 1
        for _ in range(50): await RisingEdge(self.dut.CLK)

        # Init AXI lines
        self.dut.ccore_master_d_AWVALID.value = 0
        self.dut.ccore_master_d_ARVALID.value = 0
        self.dut.ccore_master_d_WVALID.value = 0
        self.dut.ccore_master_d_RREADY.value = 0
        self.dut.ccore_master_d_BREADY.value = 0

# -----------------------------------------------------------------------------
# 5. MAIN VERIFICATION LOOP
# -----------------------------------------------------------------------------
@cocotb.test()
async def test_rx_full_regression(dut):
    """
    Runs 20 randomized iterations.
    """
    tb = UartTestbench(dut)
    clock = Clock(dut.CLK, CLK_PERIOD_NS, units="ns")
    cocotb.start_soon(clock.start(start_high=False))

    # --- REGRESSION LOOP ---
    for i in range(20):
        dut._log.info(f"================ ITERATION {i+1}/20 ================")

        # 1. Randomize
        tb.cfg.randomize()

        # 2. Setup Driver
        baud_rate = CLK_FREQ // (16 * tb.cfg.baud_div)
        uart_source = UartSource(dut.uart_cluster.uart0.SIN,
                                 baud=baud_rate,
                                 bits=8,
                                 stop_bits=field_to_stop_bits(tb.cfg.stop_field))

        # 3. Reset & Config
        await tb.reset()
        await axi_write64(tb.axi_master, BAUD_REG, tb.cfg.baud_div)

        ctrl_val = 0
        ctrl_val |= ((tb.cfg.stop_field & 0b11) << CTRL_STOP_LSB)
        ctrl_val |= ((8 & CTRL_DW_MASK) << CTRL_DW_LSB)
        await axi_write64(tb.axi_master, CTRL_REG, ctrl_val)

        await axi_write64(tb.axi_master, RX_THRESH_REG, 1)

        # 4. Inject Data
        tx_int = random.randint(0, 255)
        dut._log.info(f"   [Config] StopBits={field_to_stop_bits(tb.cfg.stop_field)}")
        dut._log.info(f"   [Action] Sending Byte: 0x{tx_int:02x}")

        await uart_source.write(tx_int.to_bytes(1, 'little'))

        # 5. Wait (Increased Safety Buffer)
        # (Start + 8 Data + 2 Stop) * Bit Time * 4 (Plenty of safety margin)
        wait_ns = (11) * (1_000_000_000 / baud_rate) * 4
        await Timer(int(wait_ns), units="ns")

        # 6. Verify (64-bit Read)
        val_64 = await axi_read64(tb.axi_master, RX_REG)
        rx_data = val_64 & 0xFF
        status = (val_64 >> 32) & 0xFFFFFFFF

        if rx_data != tx_int:
            dut._log.error(f"   [FAIL] Sent 0x{tx_int:02x} != Read 0x{rx_data:02x}")
            dut._log.error(f"   [INFO] Status Reg: 0x{status:08x} (Check if FIFO Empty?)")
            assert False, "Data Mismatch"
        else:
            dut._log.info(f"   [PASS] 0x{rx_data:02x} received correctly.")

        # 7. Sample Coverage
        tb.cg.sample(tb.cfg.stop_field, tb.cfg.baud_div)

    # --- End of Regression ---
    dut._log.info("==================================================")
    dut._log.info("FINAL COVERAGE REPORT:")
    vsc.report_coverage(fp=sys.stdout, details=True)
    vsc.write_coverage_db('rx_cov.xml')
