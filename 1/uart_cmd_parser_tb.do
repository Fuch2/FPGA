# Запуск тестбенча uart_cmd_parser в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 uart_cmd_parser   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/1; do uart_cmd_parser_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv uart_cmd_parser.sv uart_cmd_parser_tb.sv

vsim -voptargs=+acc -onfinish stop uart_cmd_parser_tb

add wave clk rst_n
add wave -radix hex data
add wave valid
add wave dut/state
add wave cmd_start cmd_stop garbage running

run -all
wave zoom full
