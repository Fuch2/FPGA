# Запуск тестбенча spi_mi_stub в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 spi_mi_stub   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/2; do spi_mi_stub_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv spi_mi_stub.sv spi_mi_stub_tb.sv

vsim -voptargs=+acc -onfinish stop spi_mi_stub_tb

add wave clk rst_n
add wave -divider "Commands"
add wave cmd_start cmd_measure cmd_stop
add wave -divider "Answer"
add wave -radix hex status data
add wave running meas_done
add wave -divider "Inside"
add wave dut/state
add wave -radix unsigned dut/cnt
add wave -radix hex dut/next_value
add wave dut/valid dut/empty

run -all
wave zoom full
