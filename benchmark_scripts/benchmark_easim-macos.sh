#!/bin/zsh
# benchmark_easim.sh - Benchmark easim simulator (macOS/Linux)

CONTROLLER="braitenberg/braitenberg_avoidance.json"
STEPS=10000
OUTFILE="benchmark_results.json"
LOGFILE="benchmark_log.txt"

echo "=== Benchmarking easim Simulator ==="
echo "Controller: $CONTROLLER"
echo "Steps: $STEPS"
echo "Output JSON: $OUTFILE"
echo "Log file: $LOGFILE"
echo "===================================="

# Run once and capture detailed log
python3 view_epuck_sim.py $CONTROLLER $STEPS false | tee $LOGFILE

# Run hyperfine for statistical benchmarking
if command -v hyperfine >/dev/null 2>&1; then
    hyperfine --warmup 2 --runs 5 \
      "python3 view_epuck_sim.py $CONTROLLER $STEPS false" \
      --export-json $OUTFILE
    echo "Hyperfine results saved to $OUTFILE"
else
    echo "Hyperfine not installed. Install with: brew install hyperfine"
fi
