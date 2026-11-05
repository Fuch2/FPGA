# Запуск тестбенча uart_pkt_parser в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 uart_pkt_parser   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/1; do uart_pkt_parser_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv uart_pkt_parser.sv uart_pkt_parser_tb.sv

vsim -voptargs=+acc -onfinish stop uart_pkt_parser_tb

add wave clk rst_n
add wave -divider "Input"
add wave -radix hex rx_data
add wave rx_valid
add wave -divider "Parser"
add wave dut/state
add wave -radix unsigned dut/data_cnt dut/tmr
add wave -radix hex dut/chk
add wave busy
add wave -divider "Output"
add wave pkt_valid chk_error timeout
add wave -radix hex meas filt err
add wave pass_valid
add wave -radix hex pass_data

run -all
wave zoom full
