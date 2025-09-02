# mipi_csi2 COCOTB VIP for mipi_csi2 protocol


# install
`pip3 install cocotbext_mipi_csi2`

#Usage

```
from cocotbext.mipi_csi2 import Mipi_csi2Bus,Mipi_csi2Driver,Mipi_csi2Config

....
class Env:
   def __init__(self,dut):
	mipi_csi2_bus = Mipi_csi2Bus(from_prefix='...',dut=....)
	mipi_csi2_config = Mipi_csi2Config()
	mipi_csi2_config.<key>=<value>
	mipi_csi2_driver = mipi_csi2Driver(mipi_csi2_bus, mipi_csi2_config)
   async def xyz(self):
 	mipi_csi2_driver.write(address,byteArray)
 	rv =mipi_csi2_driver.read(address,numbytes)
	assert rv=byteArray, "Data mismatch at %X"%(address)

