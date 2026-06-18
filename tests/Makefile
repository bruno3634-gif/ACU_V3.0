CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -lm

TEST_SRCS := $(wildcard tests/test_*.c)
TARGETS := tests/test_initial_sequence tests/test_ema_filter tests/test_autonomous_t26 \
           tests/test_ring_buffer tests/test_autonomous_functions tests/test_logger

ifneq ($(wildcard tests/test_state_machine.c),)
  TARGETS += tests/test_state_machine
endif

.PHONY: all run report clean

all: $(TARGETS)

%: %.c
	$(CC) $(CFLAGS) -o $@ $<

run: all
	@for t in $(TARGETS); do \
		echo "=== Running $$(basename $$t) ==="; \
		./$$t; \
		echo "Exit: $$?"; \
		echo ""; \
	done

report: all
	@mkdir -p tests/report
	@{ \
		echo "========================================" ; \
		echo "ACU V3.0 - Host-Based Test Report" ; \
		echo "Date: $$(date)" ; \
		echo "========================================" ; \
		echo "" ; \
		total=0; pass=0; \
		for t in $(TARGETS); do \
			name=$$(basename $$t); \
			total=$$((total + 1)); \
			echo "[$$name]"; \
			echo "  Conditions and Results:"; \
			out=$$(./$$t 2>&1); \
			ec=$$?; \
			echo "$$out" | sed 's/^/    /'; \
			if [ $$ec -eq 0 ]; then \
				pass=$$((pass + 1)); \
				echo "  Overall: PASS (exit=$$ec)"; \
			else \
				echo "  Overall: FAIL (exit=$$ec)"; \
			fi; \
			echo "  ---"; \
			echo ""; \
		done; \
		echo "========================================" ; \
		echo "SUMMARY: $$pass/$$total tests passed" ; \
		echo "========================================" ; \
	} > tests/report/test_report.txt
	@echo "Report written to tests/report/test_report.txt"
	@cat tests/report/test_report.txt

clean:
	rm -f $(TARGETS)
	rm -rf tests/report
