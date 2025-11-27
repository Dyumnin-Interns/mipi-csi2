"""Bus creator."""
import cocotb
from cocotb_bus import Bus
_signals = [
    "clk",
    "resetn",

    # Driver inputs
    "lane0_byte",
    "lane0_valid",
    "lane1_byte",
    "lane1_valid",
    "txrequest_hs",
    "txwrite_hs",
    "sop",
    "eop",

    # DUT outputs
    "txready_hs",
    "TxDDRClkHS",
    "D0",
    "D1"
]

class Mipi_csi2Bus(Bus):
      """For most cases the defaault bus creator in cocotb_bus is ok. Some protocols have edge cases that need to be handled here.

      1. Multiple names for the same signal. e.g. RDY vs not_busy
      2. relationship between signals that need to be checked e.g. byte_enable == width_of(data)/8
      3. Depending on version/profile have different lists of signals.
      """
      _signals = _signals
      def __init__(self, dut, prefix,
                   bus_separator="_", case_insensitive=False,
                   array_idx=None):
          super().__init__(entity=dut,
                           name=prefix,signals=_signals,optional_signals=[],
                           bus_separator=bus_separator,
                           case_insensitive=case_insensitive,
                           array_idx=array_idx)
