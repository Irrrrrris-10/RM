$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '..')
try {
    New-Item -ItemType Directory -Force build | Out-Null
    foreach ($test in @('rc_safety_test', 'motor_protocol_test', 'pid_test', 'belt_test')) {
        & arm-none-eabi-g++ -std=c++20 -Wall -Wextra -Werror -Iinc -c "tests/$test.cpp" -o "build/$test.o"
        if ($LASTEXITCODE -ne 0) { throw "$test failed (exit $LASTEXITCODE)" }
        Write-Output "PASS: $test (compile-time assertions)"
    }
} finally {
    Pop-Location
}
