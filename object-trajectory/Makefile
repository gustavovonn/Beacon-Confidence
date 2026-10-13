# Pure C++17 build: the engine, its tests and the cost evaluator.
#
# Deliberately a plain Makefile and not CMake. This has to build on the VisionFive 2, which runs a
# bare Linux where the only thing that can be relied on is a compiler and make. Nothing here links
# against OMNeT++, Artery or Vanetza -- the simulator-coupled part of the experiment lives in
# artery/ and is built by Artery's own CMake, never by this file.
#
#   make              build everything
#   make test         build and run the test suites
#   make cost         build and run the cost evaluator at its default scale
#   make sweep        build and run the scale sweep (the feasibility table)
#
# On the board:
#   scp -r object-trajectory/ user@visionfive:~/ && ssh user@visionfive 'cd object-trajectory && make sweep'

CXX      ?= g++
# shared/eigen is a vendored, header-only copy of Eigen 3.4.0 (see shared/eigen/PROVENANCE.md).
# -isystem rather than -I so that Eigen's own headers do not raise our warnings: they are not our
# code and we do not want to be told about them on every build.
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -isystem shared/eigen

# -MMD -MP makes the compiler emit a .d file listing every header a translation unit read, which
# is then included below. Without it, editing a header rebuilt nothing and the suites silently
# tested the previous version of the code -- which happened, and cost a confusing debugging pass.
DEPFLAGS := -MMD -MP
BUILD    := build

# Our own sources, plus the vendored assignment solver. The solver keeps its upstream .cpp
# extension because shared/scipy_lsap must stay byte-identical to what scipy ships.
ENGINE_SRC := src/UnscentedCTRA.cc src/ObjectConfidenceEngine.cc src/DatasetParser.cc
VENDOR_SRC := shared/scipy_lsap/rectangular_lsap.cpp
ENGINE_OBJ := $(ENGINE_SRC:%.cc=$(BUILD)/%.o) $(VENDOR_SRC:%.cpp=$(BUILD)/%.o)

TESTS := $(BUILD)/test_measurement_space $(BUILD)/test_pipeline $(BUILD)/test_dataset $(BUILD)/test_lab_ctra $(BUILD)/test_ukf
TOOLS := $(BUILD)/cost_evaluator $(BUILD)/experiment_runner $(BUILD)/ukf_trace

.PHONY: all test cost sweep run validate clean
all: $(TESTS) $(TOOLS)

$(BUILD)/%.o: %.cc
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(DEPFLAGS) -c -o $@ $<

$(BUILD)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(DEPFLAGS) -c -o $@ $<

$(BUILD)/test_measurement_space: tests/test_measurement_space.cc $(ENGINE_OBJ)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(DEPFLAGS) -o $@ $^

$(BUILD)/test_pipeline: tests/test_pipeline.cc $(ENGINE_OBJ)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(DEPFLAGS) -o $@ $^

$(BUILD)/test_lab_ctra: tests/test_lab_ctra.cc
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(DEPFLAGS) -o $@ $^

$(BUILD)/test_dataset: tests/test_dataset.cc $(ENGINE_OBJ)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(DEPFLAGS) -o $@ $^

$(BUILD)/test_ukf: tests/test_ukf.cc $(ENGINE_OBJ)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(DEPFLAGS) -o $@ $^

$(BUILD)/ukf_trace: tools/ukf_trace.cc $(ENGINE_OBJ)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(DEPFLAGS) -o $@ $^

$(BUILD)/cost_evaluator: tools/cost_evaluator.cc $(ENGINE_OBJ)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(DEPFLAGS) -o $@ $^

$(BUILD)/experiment_runner: tools/experiment_runner.cc $(ENGINE_OBJ)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(DEPFLAGS) -o $@ $^

test: $(TESTS)
	@./tests/run_tests.sh

cost: $(BUILD)/cost_evaluator
	@$(BUILD)/cost_evaluator

sweep: $(BUILD)/cost_evaluator
	@$(BUILD)/cost_evaluator --sweep

# Cross-validation against the reference implementations. Needs .venv (filterpy, scipy, sklearn).
validate: $(BUILD)/ukf_trace
	@./scripts/validate_against_filterpy.py

run: $(BUILD)/experiment_runner
	@mkdir -p results
	@$(BUILD)/experiment_runner --dataset dataset/groundtruth --out results/run.csv

clean:
	rm -rf $(BUILD)

# Pull in the generated dependency files. The dash keeps the first build, when none exist, quiet.
-include $(shell find $(BUILD) -name '*.d' 2>/dev/null)
