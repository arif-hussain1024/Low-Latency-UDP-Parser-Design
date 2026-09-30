# ============================================================================
# Makefile — UDP parser simulation
#
# Targets:
#   make vivado    — compile & simulate with Vivado xvlog/xelab/xsim
#   make iverilog  — compile & simulate with Icarus Verilog
#   make clean     — remove generated files
# ============================================================================

TOP     = udp_parser_tb_top
FILELIST = filelist.f

# ── Vivado (xvlog / xelab / xsim) ───────────────────────────────────────
.PHONY: vivado vivado_compile vivado_elaborate vivado_simulate

vivado: vivado_compile vivado_elaborate vivado_simulate

vivado_compile:
	xvlog -sv -f $(FILELIST) -log compile.log

vivado_elaborate:
	xelab $(TOP) -s sim_snapshot -timescale 1ns/1ps -log elaborate.log

vivado_simulate:
	xsim sim_snapshot -runall -log simulate.log

# ── Icarus Verilog ───────────────────────────────────────────────────────
.PHONY: iverilog

iverilog:
	iverilog -g2012 -o sim.vvp -f $(FILELIST)
	vvp sim.vvp

# ── Clean ────────────────────────────────────────────────────────────────
.PHONY: clean

clean:
	rm -rf xsim.dir sim_snapshot.wdb
	rm -f *.log *.pb *.jou *.wdb *.vcd *.vvp
	rm -f webtalk*.* xvlog.* xelab.* xsim.*
