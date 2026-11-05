<#
.SYNOPSIS
    Компиляция и запуск тестбенча модуля в ModelSim.

.DESCRIPTION
    Для модуля <Module> выполняет скрипт <Module>_tb.do из папки этого файла.
    Соглашение: .do-скрипт сам компилирует нужные файлы (vlog) и запускает
    симуляцию (vsim ... run -all), а тестбенч в конце печатает строку
    "TEST PASSED" или "TEST FAILED".

    Без -Gui: пакетный режим (vsim -c), полный лог в sim_<Module>.log,
    код возврата 0 = PASS, 1 = FAIL или нет итога, 2 = ошибка запуска.
    С -Gui: открывает ModelSim с временными диаграммами и не ждёт его закрытия.

.EXAMPLE
    .\run_sim.ps1 UART
    .\run_sim.ps1 UART -Gui
    .\run_sim.ps1 -List
#>
param(
    [Parameter(Position = 0)]
    [string]$Module,
    [switch]$Gui,
    [switch]$List,
    [string]$ModelSimDir = 'C:\intelFPGA\18.1\modelsim_ase\win32aloem'
)

$vsim = Join-Path $ModelSimDir 'vsim.exe'
if (-not (Test-Path $vsim)) {
    Write-Host "Не найден $vsim (параметр -ModelSimDir)" -ForegroundColor Red
    exit 2
}

Push-Location $PSScriptRoot
try {
    # список доступных тестбенчей
    $available = Get-ChildItem -Filter '*_tb.do' | ForEach-Object { $_.Name -replace '_tb\.do$', '' }
    if ($List -or -not $Module) {
        Write-Host "Тестбенчи в $PSScriptRoot :"
        $available | ForEach-Object { Write-Host "  $_" }
        Write-Host "Запуск: .\run_sim.ps1 <модуль> [-Gui]"
        exit 0
    }

    $do = "${Module}_tb.do"
    if (-not (Test-Path $do)) {
        Write-Host "Нет файла $do. Доступные: $($available -join ', ')" -ForegroundColor Red
        exit 2
    }

    if ($Gui) {
        Start-Process -FilePath $vsim -ArgumentList '-do', $do
        Write-Host "ModelSim открыт: $do"
        exit 0
    }

    # пакетный режим: после остановки на $finish или ошибки в .do-скрипте
    # выполняется quit, итог определяется по логу
    $log = "sim_${Module}.log"
    $cmd = "onbreak {resume}; do $do; quit -f"
    $t0  = Get-Date
    & $vsim -c -l $log -do $cmd | Out-Host
    $code = $LASTEXITCODE

    # если vsim.wlf занят, ModelSim пишет диаграммы во временный wlft*; убираем созданные этим запуском
    Get-ChildItem -Filter 'wlft*' -File | Where-Object { $_.LastWriteTime -ge $t0 } | Remove-Item -ErrorAction SilentlyContinue

    $errors = @(Select-String -Path $log -Pattern '^\*\* (Error|Fatal)|^# \*\* (Error|Fatal)')
    $failed = Select-String -Path $log -Pattern 'TEST FAILED' -Quiet
    $passed = Select-String -Path $log -Pattern 'TEST PASSED' -Quiet

    Write-Host ''
    if ($failed) {
        Write-Host "$Module : FAIL (лог $log)" -ForegroundColor Red
        exit 1
    } elseif ($passed -and $errors.Count -eq 0) {
        Write-Host "$Module : PASS" -ForegroundColor Green
        exit 0
    } else {
        Write-Host "$Module : нет итога TEST PASSED, ошибок: $($errors.Count), код vsim: $code (лог $log)" -ForegroundColor Red
        $errors | Select-Object -First 10 | ForEach-Object { Write-Host "  $($_.Line)" -ForegroundColor Red }
        exit 1
    }
}
finally {
    Pop-Location
}
