#!/bin/bash
# ACU V3.0 Test Runner — compiles and runs all host-based unit tests
# Usage: cd /path/to/ACU_V3.0 && ./tests/run_tests.sh
# No make required — compiles directly with gcc.

cd "$(dirname "$0")/.." || exit 1
mkdir -p tests/report

CC=gcc
CFLAGS="-Wall -Wextra -std=c11 -lm"

report="tests/report/test_report.txt"
> "$report"

TEST_SRCS=(
    tests/test_adc_callbacks.c
    tests/test_autonomous_functions.c
    tests/test_autonomous_t26.c
    tests/test_ble_config.c
    tests/test_can_queue.c
    tests/test_ema_filter.c
    tests/test_initial_sequence.c
    tests/test_logger.c
    tests/test_ring_buffer.c
    tests/test_state_machine.c
    tests/test_temperature.c
    tests/test_tim_callback.c
    tests/test_uart_callbacks.c
)

echo "========================================"
echo "ACU V3.0 - Host-Based Test Suite"
echo "========================================"
echo ""

{
echo "========================================"
echo "ACU V3.0 - Host-Based Test Report"
echo "Date: $(date)"
echo "========================================"
echo ""
} >> "$report"

total=0
passed=0
failed=0

for src in "${TEST_SRCS[@]}"; do
    [ -f "$src" ] || continue

    name=$(basename "$src" .c)
    bin="tests/${name}"

    echo "--- Building: ${name} ---"

    # Compile (capture stderr too)
    compile_out=$( $CC $CFLAGS -o "$bin" "$src" 2>&1 ) && compile_ok=1 || compile_ok=0

    if [ "$compile_ok" = "1" ]; then
        echo "  Compilation: OK"
    else
        echo "  Compilation: FAILED"
        echo "$compile_out"
        {
        echo "[${name}]"
        echo "  Build: FAILED (compilation error)"
        echo "  Compiler output:"
        echo "$compile_out" | sed 's/^/    /'
        echo "  ---"
        echo ""
        } >> "$report"
        total=$((total + 1))
        failed=$((failed + 1))
        continue
    fi

    # Run (capture stdout+stderr; do not let set -e kill us on non-zero exit)
    echo "  Running: ./${bin}"
    set +e
    output=$( ./"$bin" 2>&1 )
    ec=$?
    set -e 2>/dev/null || true

    total=$((total + 1))

    {
    echo "[${name}]"
    echo "  Conditions and Results:"
    echo "$output" | sed 's/^/    /'
    if [ $ec -eq 0 ]; then
        passed=$((passed + 1))
        echo "  Overall: PASS (exit=${ec})"
    else
        failed=$((failed + 1))
        echo "  Overall: FAIL (exit=${ec})"
    fi
    echo "  ---"
    echo ""
    } >> "$report"

    if [ $ec -eq 0 ]; then
        echo "  Result: PASS"
    else
        echo "  Result: FAIL (exit=${ec})"
    fi
    echo ""
done

{
echo "========================================"
echo "SUMMARY: ${passed}/${total} tests passed"
echo "========================================"
} >> "$report"

echo "========================================"
echo "SUMMARY: ${passed}/${total} tests passed"
echo "========================================"
echo ""
echo "Full report: ${report}"

if [ $failed -ne 0 ]; then
    exit 1
fi
