"""Bus creator."""
import cocotb
from cocotb_bus import Bus
_signals = [
    "clk",
    "resetn",
    "lane0_byte",
    "lane0_valid",
    "lane1_byte",
    "lane1_valid",
    "txrequest_hs",
    "txwrite_hs",
    "sop",
    "eop",
    "txready_hs",
    "TxDDRClkHS",
    "D0",
    "D1"
]

class Mipi_csi2Bus(Bus):
    def __init__(self, dut, prefix,
                 bus_separator="_", case_insensitive=False,
                 array_idx=None):
        super().__init__(entity=dut,
                         name=prefix,
                         signals=_signals,
                         optional_signals=[],
                         bus_separator=bus_separator,
                         case_insensitive=case_insensitive,
                         array_idx=array_idx)
