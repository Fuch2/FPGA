# Запуск тестбенча uart_tx_mux в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 uart_tx_mux   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/1; do uart_tx_mux_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv uart_tx_mux.sv uart_tx_mux_tb.sv

vsim -voptargs=+acc -onfinish stop uart_tx_mux_tb

add wave clk rst_n
add wave -divider "Sources"
add wave -radix hex in_data
add wave -radix binary in_valid in_last in_ready
add wave -divider "Mux"
add wave dut/locked
add wave -radix unsigned dut/owner dut/sel
add wave -divider "Output"
add wave -radix hex out_data
add wave out_valid out_ready out_last

run -all
wave zoom full
