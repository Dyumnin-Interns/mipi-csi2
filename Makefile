TOPLEVEL_LANG   = verilog
SIM             = icarus
MODULE          = test_dut
WAVES           = 1
VERILOG_SOURCES = $(PWD)/rtl/dut_top.v \
                  $(PWD)/rtl/dphy_model.v
TOPLEVEL        = dut_top
include $(shell cocotb-config --makefiles)/Makefile.sim
