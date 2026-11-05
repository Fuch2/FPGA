# Запуск тестбенча uart_ind в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 uart_ind   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/1; do uart_ind_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv seg7_dynamic.sv uart_ind.sv uart_ind_tb.sv

vsim -voptargs=+acc -onfinish stop uart_ind_tb

add wave clk rst_n
add wave -divider "Events"
add wave rx_byte
add wave -radix hex rx_data
add wave tx_byte rx_pkt tx_pkt garbage
add wave err_frame err_chk err_timeout err_ovf running clear next_mode
add wave -divider "Inside"
add wave dut/mode
add wave -radix hex dut/last_byte dut/cnt_rx_pkt dut/cnt_tx_pkt dut/cnt_err dut/cnt_garbage
add wave -divider "Board"
add wave -radix binary dig seg led

run -all
wave zoom full
