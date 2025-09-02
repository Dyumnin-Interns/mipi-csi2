"""Bus creator."""
import cocotb
from cocotb_bus import Bus


class Mipi_csi2Bus(Bus):
      """For most cases the defaault bus creator in cocotb_bus is ok. Some protocols have edge cases that need to be handled here.

      1. Multiple names for the same signal. e.g. RDY vs not_busy
      2. relationship between signals that need to be checked e.g. byte_enable == width_of(data)/8
      3. Depending on version/profile have different lists of signals.
      """
      _signals:list[str]=[]
      def __init__(self, dut, prefix,
                   bus_separator="_", case_insensitive=False,
                   array_idx=None):
          super().__init__(entity=dut,
                           name=prefix,signals=_signals,optional_signals=[],
                           bus_separator=bus_separator,
                           case_insensitive=case_insensitive,
                           array_idx=array_idx)
