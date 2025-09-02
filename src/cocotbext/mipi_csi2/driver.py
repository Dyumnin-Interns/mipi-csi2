from .config import default_config
class Mipi_csi2Driver:

        def __init__(self,bus,config=default_config,name=None):
               self.bus=bus
               self.config=config
               pass

        async def write(self,address:int, data:bytes):
                pass

        async def read(self,address:int,numBytes:int):
                pass

        async def _txrx(self):
                pass

        def add_callback(self, compare_fn):
                """Callback into scoreboard."""
                pass

