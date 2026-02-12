import os
import random
import logging
from enum import Enum
import cocotb
import vsc

CLK_PERIOD_NS = 100                 # 10 MHz Clock
CLK_FREQ      = 1_000_000_000 // CLK_PERIOD_NS
UART_BASE     = 0x00011300          # Base Address

# -----------------------------------------------------------------------------
# 2. REGISTER MAP & CONSTANTS
# -----------------------------------------------------------------------------
UartParity = Enum("UartParity", "NONE EVEN ODD MARK SPACE")

BAUD_REG      = UART_BASE + 0x00
TX_REG        = UART_BASE + 0x04
RX_REG        = UART_BASE + 0x08
STATUS_REG    = UART_BASE + 0x0C
DELAY_REG     = UART_BASE + 0x10
CTRL_REG      = UART_BASE + 0x14
INTERRUPT_EN  = UART_BASE + 0x18
IQCYC_REG     = UART_BASE + 0x1C
RX_THRESH     = UART_BASE + 0x20

# Control Register Shifts
CTRL_STOP_LSB   = 1
CTRL_PARITY_LSB = 3
CTRL_DW_LSB     = 5
CTRL_DW_MASK    = 0x3

def parity_to_field(p: UartParity) -> int:
    if p == UartParity.ODD:  return 0b01
    if p == UartParity.EVEN: return 0b10
    return 0b00

def field_to_stop_bits(field: int):
    if field == 0b01: return 1.5
    if field == 0b10: return 2
    return 1

# --- HELPER TO MAP INT WIDTH TO BINARY CODE ---
def get_width_code(width):
    if width == 8: return 0b11
    if width == 7: return 0b10
    if width == 6: return 0b01
    return 0b00

# -----------------------------------------------------------------------------
# 3. HELPER FUNCTIONS
# -----------------------------------------------------------------------------
async def axi_write32(axim, addr: int, value: int, *, awid=1):
    # Lazy Import
    from cocotbext.axi import AxiBurstType

    data = int(value & 0xFFFFFFFF).to_bytes(4, "little")
    await axim.write(address=addr, data=data,
                     awid=awid, burst=AxiBurstType.FIXED, size=2)

async def rmw16(axim, reg_addr: int, value16: int):
    aligned_addr = reg_addr & ~0x3
    is_upper = (reg_addr & 0x2) != 0
    shift = 16 if is_upper else 0
    val32 = (value16 & 0xFFFF) << shift
    await axi_write32(axim, aligned_addr, val32)

# -----------------------------------------------------------------------------
# 4. WAVEFORM AUTOMATION CHECKER
# -----------------------------------------------------------------------------
async def verify_uart_waveform(dut, expected_data, baud_rate, data_width):
    # --- SAFE LOCAL IMPORTS ---
    from cocotb.triggers import FallingEdge, Timer

    """
    Manual Waveform Monitor: Checks physical pins for timing and data integrity.
    """
    bit_period_ns = 1_000_000_000 / baud_rate
    dut._log.info(f"[Waveform Check] Starting Monitor | Baud: {baud_rate} | Width: {data_width}")

    # 1. Wait for Start Bit (Falling Edge)
    await FallingEdge(dut.uart_cluster.uart0.SOUT)

    # 2. Check Start Bit Stability (Sample at 50% of bit period)
    await Timer(bit_period_ns / 2, unit='ns')
    if dut.uart_cluster.uart0.SOUT.value != 0:
        raise RuntimeError("Waveform Error: Start bit did not hold LOW!")

    # 3. Read Data Bits
    rec_val = 0
    for i in range(data_width):
        await Timer(bit_period_ns, unit='ns')
        bit_val = int(dut.uart_cluster.uart0.SOUT.value)
        rec_val |= (bit_val << i) # LSB First

    # 4. Check Stop Bit
    await Timer(bit_period_ns, unit='ns')
    if dut.uart_cluster.uart0.SOUT.value != 1:
        dut._log.error(f"Waveform Error: Stop bit missing. Last Data Bit was {bit_val}")
        raise RuntimeError("Waveform Error: Stop bit missing (Line should be HIGH)!")

    # 5. Verify
    dut._log.info(f"[Waveform Check] Captured: {hex(rec_val)} | Expected: {hex(expected_data)}")
    if rec_val != expected_data:
        raise RuntimeError(f"Waveform Mismatch! Waveform: {hex(rec_val)} != Expected: {hex(expected_data)}")

    dut._log.info("[Waveform Check] PASSED ✓")

# -----------------------------------------------------------------------------
# 5. COVERAGE MODEL
# -----------------------------------------------------------------------------
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

# -----------------------------------------------------------------------------
# 6. TESTBENCH CLASSES
# -----------------------------------------------------------------------------
class Testbench:
    def __init__(self, dut):
        # Lazy Import
        from cocotbext.axi import AxiMaster, AxiBus

        self.dut = dut
        self.log = logging.getLogger("cocotb.tb")
        self.log.setLevel(logging.INFO)
        self.axi_master = AxiMaster(AxiBus.from_prefix(dut, 'ccore_master_d'),
                                    clock=dut.CLK, reset=dut.RST_N, reset_active_level=False)
        self.cg = my_covergroup()

class uart_components:
    def __init__(self, dut, clk_freq, axi_baud_value, stop_bits_num, selected_parity, data_width, uart_base_addr):
        # Lazy Import
        from cocotbext.uart import UartSink

        # --- FIX: Convert Enum to None/String for VIP ---
        vip_parity = None
        if selected_parity == UartParity.ODD:  vip_parity = "ODD"
        if selected_parity == UartParity.EVEN: vip_parity = "EVEN"

        self.baud_rate = clk_freq // (16 * axi_baud_value)
        if uart_base_addr == 0x00011300:
            self.uart_tx = UartSink(dut.uart_cluster.uart0.SOUT, baud=self.baud_rate,
                                    bits=data_width, stop_bits=stop_bits_num, parity=vip_parity)
        else:
            raise ValueError(f"Unknown UART Address: {hex(uart_base_addr)}")

# -----------------------------------------------------------------------------
# 7. MAIN TEST
# -----------------------------------------------------------------------------
@cocotb.test()
async def test_64bit_native_verification(dut):
    # --- SAFE LOCAL IMPORTS ---
    from cocotb.clock import Clock
    from cocotb.triggers import RisingEdge

    """
    Combines robust verification (Coverage/Random Config) with
    Native 64-bit Transaction testing.
    """

    # --- A. Setup Phase ---
    clock = Clock(dut.CLK, CLK_PERIOD_NS, unit="ns")
    cocotb.start_soon(clock.start(start_high=False))

    dut.RST_N.value = 0
    for _ in range(200): await RisingEdge(dut.CLK)
    dut.RST_N.value = 1
    for _ in range(50): await RisingEdge(dut.CLK)

    # Silence CPU
    dut.ccore_master_d_AWVALID.value = 0
    dut.ccore_master_d_ARVALID.value = 0
    dut.ccore_master_d_WVALID.value = 0
    dut.ccore_master_d_RREADY.value = 0
    dut.ccore_master_d_BREADY.value = 0

    for _ in range(20): await RisingEdge(dut.CLK)
    tb = Testbench(dut)

    # --- B. Configuration Phase ---
    await rmw16(tb.axi_master, DELAY_REG, 0x0000)
    await rmw16(tb.axi_master, IQCYC_REG, 0x0000)
    await rmw16(tb.axi_master, RX_THRESH, 0x0000)
    await rmw16(tb.axi_master, INTERRUPT_EN, 0x0000)

    # Generate Random Config
    stop_field = random.choice([0b00, 0b01, 0b10, 0b11])
    stop_bits_num = field_to_stop_bits(stop_field)
    parity_sel = UartParity.NONE
    parity_field = parity_to_field(parity_sel)

    data_width = 8

    # --- FIXED: Use Code mapping instead of raw integer ---
    dw_code = get_width_code(data_width)

    ctrl_val = 0
    ctrl_val |= ((stop_field & 0b11) << CTRL_STOP_LSB)
    ctrl_val |= ((parity_field & 0b11) << CTRL_PARITY_LSB)
    ctrl_val |= ((dw_code & CTRL_DW_MASK) << CTRL_DW_LSB)

    dut._log.info(f"Configuring CTRL: Width={data_width} (Code={bin(dw_code)}), Stop={stop_bits_num}")
    await rmw16(tb.axi_master, CTRL_REG, ctrl_val)

    # --- C. Transaction Phase ---
    baud_val = 0x0005
    etx_data = random.getrandbits(data_width)
    tx_char  = etx_data & 0xFF
    actual_baud_rate = CLK_FREQ // (16 * baud_val)

    # Setup Standard Library Monitor
    tb1 = uart_components(dut, CLK_FREQ, baud_val,
                          stop_bits_num, parity_sel, data_width, UART_BASE)
    tb.cg.sample(0, stop_field, parity_field, data_width)

    # *** START THE MANUAL WAVEFORM CHECKER ***
    waveform_task = cocotb.start_soon(
        verify_uart_waveform(dut, tx_char, actual_baud_rate, data_width)
    )

    # Prepare Data
    data_64bit_int = (tx_char << 32) | baud_val
    data_bytes = data_64bit_int.to_bytes(8, 'little')

    dut._log.info("---------------------------------------------------")
    dut._log.info(f" EXECUTING 64-BIT WRITE TO {hex(UART_BASE)}")
    dut._log.info(f" Payload: {data_bytes.hex()}")
    dut._log.info(f"   -> Baud: {hex(baud_val)} (Approx {actual_baud_rate} Hz)")
    dut._log.info(f"   -> TX  : {hex(tx_char)}")
    dut._log.info("---------------------------------------------------")

    try:
        await tb.axi_master.write(address=UART_BASE, data=data_bytes, size=3)
        dut._log.info("AXI Write Sent.")
    except Exception as e:
        dut._log.warning(f"AXI Error (Expected SLVERR): {e}")

    # --- D. Verification Phase ---
    await waveform_task
    received = await tb1.uart_tx.read(count=1)
    rx_byte = int(received[0])

    tb.cg.sample(rx_byte, stop_field, parity_field, data_width)
    dut._log.info(f"Sent: {hex(tx_char)}, Recv: {hex(rx_byte)}")
    assert rx_byte == tx_char, f"Mismatch: Sent {hex(tx_char)} != Recv {hex(rx_byte)}"

    vsc.write_coverage_db('cov.xml')
    dut._log.info("Test Passed & Coverage Dumped.")
