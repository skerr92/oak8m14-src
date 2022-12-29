# SPDX-FileCopyrightText: © 2022 Seth Kerr <hello@oakdev.tech>
# Based on SPELL by Uri Shaked <https://github.com/wokwi/verispell>
# SPDX-License-Identifier: MIT

export COCOTB_REDUCED_LOG_FMT=1
export LIBPYTHON_LOC=$(shell cocotb-config --libpython)

all: test_oak8m14

test_execute:
	iverilog -I src -o execute_tb.out test/execute_tb.v src/program.v
	./execute_tb.out
	gtkwave program_tb.vcd test/execute_tb.gtkw

test_pmem_dff:
	iverilog -I src -o pmem_dff_tb.out test/assert.v test/pmem_dff_tb.v src/pmem_dff.v
	./pmem_dff_tb.out
	gtkwave pmem_dff_tb.vcd test/pmem_dff_tb.gtkw

test_oak8m14:
	iverilog -I src -s oak8m -s dump -D DFF_DELAY -o oak8m14_test.out src/cpu.v src/pmem.v src/pmem_dff.v src/pmem_io.v src/program.v test/dump_oak8m.v
	MODULE=test.test_oak8m14 vvp -M $$(cocotb-config --prefix)/cocotb/libs -m libcocotbvpi_icarus ./oak8m14_test.out

test_oak8m14_show: test_oak8m14
	gtkwave oak8m_test.vcd test/oak8m14_test.gtkw

test_gate_level:
	iverilog -o oak8m14_gate_level.out -s oak8m -s dump -g2012 gl/oak8m14.lvs.powered.v test/dump_oak8m.v -I $(PDK_ROOT)/sky130A
	MODULE=test.test_oak8m14 vvp -M $$(cocotb-config --prefix)/cocotb/libs -m libcocotbvpi_icarus oak8m14_gate_level.out
	gtkwave oak8m_test.vcd test/oak8m14_test.gtkw

format:
	verible-verilog-format --inplace src/*.v test/*.v