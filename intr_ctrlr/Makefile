SHELL := /bin/bash

PYTHON ?= python3
RTL ?= intr_ctrl.v
FILELIST ?= filelist.f
TESTS ?= cocotb_bench.py
BUILD_DIR ?= build/cocotb
REPORT_DIR ?= reports
COVERAGE_FILE ?= $(REPORT_DIR)/coverage.json


.PHONY: test clean

test:
	mkdir -p $(BUILD_DIR) $(REPORT_DIR)
	$(PYTHON) scripts/run_cocotb.py \
		--rtl $(RTL) \
		--tests $(TESTS) \
		--filelist $(FILELIST) \
		--build-dir $(BUILD_DIR) \
		--coverage-file $(COVERAGE_FILE) \
		--results-xml $(REPORT_DIR)/results.xml

clean:
	rm -rf build reports
