#===============================================================================
# Config
#-------------------------------------------------------------------------------
# use C2RTL riscvcore
#C2RTL               := 1

PROC                := riscvultrasmall-32i
#PROC                := riscvcorep-32i

SIM                 := iverilog
#SIM                 := verilator

SIMRV               := 

# simulation time out (cycle)
TIMEOUT             := 50000
#TIMEOUT             := 1000

# generate rslt.txt
RSLT                := 1
RSLT_FILE           := rslt.txt

# generate trace.txt
TRACE               := 1
TRACE_FILE          := ultratrace.txt
#-----------------------------------------|-----------------------------------------#
# count pc ir                             | count pc ir                             #
# zero  ra   sp   gp   tp   t0   t1   t2  |  x0   x1   x2   x3   x4   x5   x6   x7  #
#  s0   s1   a0   a1   a2   a3   a4   a5  |  x8   x9   x10  x11  x12  x13  x14  x15 #
#  a6   a7   s2   s3   s4   s5   s6   s7  |  x16  x17  x18  x19  x20  x21  x22  x23 #
#  s8   s9   s10  s11  t3   t4   t5   t6  |  x24  x25  x26  x27  x28  x29  x30  x31 #
#-----------------------------------------|-----------------------------------------#

# generate debug.txt
#DEBUG               := 1
DEBUG_FILE          := debug.txt

# diff all riscv-tests traces
# command: make isa
# Note: Make sure the C2RTL flag in config.vh and Makefile is commented out
DIFF_TRACE          := 1

# Vivado version
VIVADO_VER          := 2022.1

#==============================================================================
# Architecture
#------------------------------------------------------------------------------
# defined in $(src_dir)/Makefile.in
ILEN                :=
XLEN                :=
ARCH                :=
ABI                 :=

#===============================================================================
# Sources
#-------------------------------------------------------------------------------
src_dir             := src/$(PROC)
# defined in $(src_dir)/Makefile.in
srcs                :=
include $(src_dir)/Makefile.in

include_dir         := $(src_dir)
VPPFLAGS            := $(addprefix -I, $(include_dir))

#===============================================================================
# Directory
#-------------------------------------------------------------------------------
prog_dir            := prog
riscv-tests_dir     := $(prog_dir)/riscv-tests
isa_dir             := $(riscv-tests_dir)/isa
test_dir            := $(prog_dir)/test/$(ARCH)

trace_dir           := trace

#===============================================================================
# Verilator
#-------------------------------------------------------------------------------
# Verilog
#top_module          := m_verilator_testbench
top_module          := testbench
clk                 := clk

# Verilator
vl_dir              := verilator
vl_src              := $(vl_dir)/sim_main.cpp
vl_prefix           := Vtop

#-------------------------------------------------------------------------------
# Verilator simulation flags
#-------------------------------------------------------------------------------
FLAGS  = -DNO_IP -DNO_SERIAL -DSERIAL_WAIT_CNT=2

ifeq ($(SIM), iverilog)
FLAGS += -o $(vl_prefix)
endif
ifeq ($(SIM), verilator)
FLAGS += -O2
#FLAGS += -Wall
FLAGS += --cc
FLAGS += --top-module $(top_module)
FLAGS += --clk $(clk)
FLAGS += --prefix $(vl_prefix)
FLAGS += --exe $(vl_src)
#FLAGS += --build
endif

#-------------------------------------------------------------------------------
ifdef C2RTL
FLAGS += -DC2RTL=$(C2RTL)
endif

ifdef TIMEOUT
FLAGS += -DTIMEOUT=$(TIMEOUT)
endif

ifdef TRACE
FLAGS += -DTRACE
TRACE_FILE ?= trace.txt
FLAGS += -DTRACE_FILE=\"$(TRACE_FILE)\"
endif

ifdef DEBUG
FLAGS += -DDEBUG
DEBUG_FILE ?= debug.txt
FLAGS += -DDEBUG_FILE=\"$(DEBUG_FILE)\"
endif

#-------------------------------------------------------------------------------
ifdef ISA
FLAGS += -DIMEMFILE=\"$(IMEMFILE)\" -DDMEMFILE=\"${DMEMFILE}\" -DMEMSIZE=512*4
endif

ifdef TEST
FLAGS += -DIMEMFILE=\"$(IMEMFILE)\" -DDMEMFILE=\"${DMEMFILE}\" -DMEMSIZE=1024*4
endif

#===============================================================================
# Build rules
#-------------------------------------------------------------------------------
.PHONY: default
#default: ;
default: sim run

#-------------------------------------------------------------------------------
.PHONY: sim run
sim:
	@$(SIM) $(VPPFLAGS) $(FLAGS) $(srcs)
ifeq ($(SIM), verilator)
	@$(MAKE) -j -C obj_dir -f $(vl_prefix).mk $(vl_prefix) --no-print-directory
endif

run:
ifeq ($(SIM), iverilog)
	@./$(vl_prefix)
endif
ifeq ($(SIM), verilator)
	@obj_dir/$(vl_prefix)
endif

#-------------------------------------------------------------------------------
.PHONY: clean rsltclean traceclean txtclean vivadoclean distclean
clean:
	$(RM) -r obj_dir

rsltclean:
	@$(RM) $(RSLT_FILE)

traceclean:
	@$(RM) $(trace_dir)/*.txt

txtclean:
	$(RM) *.txt

vivadoclean:
	$(RM) *.jou *.log *.str
	$(RM) -r $(shell find vivado \
		-name '*.cache'         -or \
		-name '*.hw'            -or \
		-name '*.runs'          -or \
		-name '*.sim'           -or \
		-name '*.ip_user_files')

distclean: clean traceclean txtclean vivadoclean

#-------------------------------------------------------------------------------
.PHONY: check
check:
	@echo

#===============================================================================
# Vivado
#------------------------------------------------------------------------------
VIVADO_VER          ?= 2022.1
UART_DEVICE         ?= ttyUSB1

.PHONY: vivado gtkterm send

vivado: SHELL=/bin/bash
vivado:
	source /tools/Xilinx/Vivado/$(VIVADO_VER)/settings64.sh && \
	vivado &

gtkterm:
	gtkterm -p /dev/$(UART_DEVICE) -s 1000000 &

send:
	cat $(test_dir)/test.bin > /dev/$(UART_DEVICE)

#===============================================================================
# riscv-tests/isa
#------------------------------------------------------------------------------
# fence_i
rv32ui_sc_tests := \
	simple \
	add addi sub and andi or ori xor xori \
	sll slli srl srli sra srai slt slti sltiu sltu \
	beq bge bgeu blt bltu bne jal jalr \
	sb sh sw lb lbu lh lhu lw \
	auipc lui

rv32ui_p_tests := $(addprefix rv32ui-p-, $(rv32ui_sc_tests))

$(rv32ui_sc_tests): %: rv32ui-p-%

ifdef DIFF_TRACE
isa: traceclean
endif
isa: rsltclean $(rv32ui_sc_tests)

ifdef RSLT
$(rv32ui_p_tests): output := | tee -a $(RSLT_FILE)
endif
$(rv32ui_p_tests):
ifdef DIFF_TRACE
# simrv
	@echo $@ simrv                                                         $(output)
	@echo ---------------------------------------------------------------- $(output)
	@simrv -a -m $(isa_dir)/$@.bin -t 0 5000                               $(output)
	@mv trace.txt $(trace_dir)/$@_simrv_trace.txt                          $(output)
	@echo                                                                  $(output)
	@echo                                                                  $(output)
# ultrasmall
	@echo $@ ultrasmall                                                    $(output)
	@echo ---------------------------------------------------------------- $(output)
	@$(MAKE) sim ISA=1 IMEMFILE="$(isa_dir)/$@.$(ILEN).hex" DMEMFILE="$(isa_dir)/$@.$(XLEN).hex" \
	TRACE=1 TRACE_FILE=$(trace_dir)/$@_ultrasmall_trace.txt > /dev/null
	@$(MAKE) run --no-print-directory                                      $(output)
	@echo                                                                  $(output)
	@echo                                                                  $(output)
# diff trace simrv ultrasmall
	@diff $(trace_dir)/$@_simrv_trace.txt $(trace_dir)/$@_ultrasmall_trace.txt >> diff.txt
else #ifndef DIFF_TRACE
	@echo $@                                                               $(output)
	@echo ---------------------------------------------------------------- $(output)
	@$(MAKE) sim ISA=1 IMEMFILE="$(isa_dir)/$@.$(ILEN).hex" DMEMFILE="$(isa_dir)/$@.$(XLEN).hex" > /dev/null
	@$(MAKE) run --no-print-directory                                      $(output)
	@echo                                                                  $(output)
	@echo                                                                  $(output)
endif

#===============================================================================
# test
#------------------------------------------------------------------------------
.PHONY: test
test:
	@echo $@
	@echo ----------------------------------------------------------------
	@$(MAKE) sim TEST=1 IMEMFILE="$(test_dir)/$@.$(ILEN).hex" DMEMFILE="$(test_dir)/$@.$(XLEN).hex" > /dev/null
	@$(MAKE) run --no-print-directory
	@echo