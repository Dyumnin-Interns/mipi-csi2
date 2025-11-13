import cocotb
from cocotb.triggers import RisingEdge

async def check_hs_entry_exit_timing(dut, min_settle=6, min_trail=2, min_exit=4):
    # Wait for HS request
    while int(dut.txrequest_hs.value) == 0:
        await RisingEdge(dut.clk)
    # Measure HS-SETTLE until txready_hs rises
    settle = 0
    while int(dut.txready_hs.value) == 0:
        settle += 1
        await RisingEdge(dut.clk)
    assert settle >= min_settle, f"HS-SETTLE short: {settle} < {min_settle}"

    # HS clock must toggle during HS
    toggles = 0
    last = int(dut.TxDDRClkHS.value)
    for _ in range(12):
        await RisingEdge(dut.clk)
        cur = int(dut.TxDDRClkHS.value)
        if cur != last:
            toggles += 1
            last = cur
    assert toggles > 0, "TxDDRClkHS did not toggle during HS"

    # Measure TRAIL: after request low until ready low
    while int(dut.txrequest_hs.value) == 1:
        await RisingEdge(dut.clk)
    trail = 0
    while int(dut.txready_hs.value) == 1:
        trail += 1
        await RisingEdge(dut.clk)
    assert trail >= min_trail, f"HS-TRAIL short: {trail} < {min_trail}"

    # Measure EXIT by observing clock stabilization after HS
    stable = 0
    lastc = int(dut.TxDDRClkHS.value)
    exit_cnt = 0
    while stable < 4 and exit_cnt < 64:
        await RisingEdge(dut.clk)
        cur = int(dut.TxDDRClkHS.value)
        if cur == lastc:
            stable += 1
        else:
            stable = 0
            lastc = cur
        exit_cnt += 1
    assert exit_cnt >= min_exit, f"HS-EXIT short: {exit_cnt} < {min_exit}"

def ddr_low2bits(byte_val: int):
    """Return (bit1, bit0) to match D0/D1 [1:0] packing."""
    return ((byte_val >> 1) & 1, byte_val & 1)

async def spot_check_serialization(dut, lane_name="D0", samples=8):
    """Collect 'samples' packed 2-bit DDR values while writes are accepted."""
    sig = getattr(dut, lane_name)
    obs = []
    while len(obs) < samples:
        await RisingEdge(dut.clk)
        if int(dut.txwrite_hs.value) and int(dut.txready_hs.value):
            obs.append(int(sig.value) & 0x3)
    return obs
