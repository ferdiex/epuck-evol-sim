# benchmark_easim.ps1 - Benchmark easim simulator (Windows PowerShell)

$controller = "braitenberg/braitenberg_avoidance.json"
$steps = 10000
$jsonfile = "benchmark_results.json"

Write-Host "=== Benchmarking easim Simulator (Windows) ==="
Write-Host "Controller: $controller"
Write-Host "Steps: $steps"
Write-Host "=============================================="

# Measure execution time directly
$time = Measure-Command {
    python view_epuck_sim.py $controller $steps false
}

# Capture Python process for CPU/memory
$proc = Get-Process -Name python | Select-Object -First 1
$cpu = $proc.CPU
$mem = [math]::Round($proc.WorkingSet64 / 1MB, 2)

# Calculate metrics
$avg_time_per_step = $time.TotalSeconds / $steps
$throughput = $steps / $time.TotalSeconds

# Save results in JSON
$result = @{
    Controller       = $controller
    Steps            = $steps
    TotalSeconds     = $time.TotalSeconds
    AvgTimePerStep   = $avg_time_per_step
    ThroughputStepsS = $throughput
    CPU              = $cpu
    MemoryMB         = $mem
}
$result | ConvertTo-Json | Out-File $jsonfile -Encoding utf8

Write-Host "Benchmark results saved to $jsonfile"
Write-Host "Tiempo total: $($time.TotalSeconds) s"
Write-Host "Tiempo promedio por paso: $avg_time_per_step s"
Write-Host "Throughput: $throughput pasos/s"

