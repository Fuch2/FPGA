# Запуск тестбенча uart_pkt_tx в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 uart_pkt_tx   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/1; do uart_pkt_tx_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv uart_pkt_tx.sv uart_pkt_tx_tb.sv

vsim -voptargs=+acc -onfinish stop uart_pkt_tx_tb

add wave clk rst_n
add wave -divider "Input"
add wave -radix hex meas filt err
add wave in_valid in_ready
add wave -divider "Output"
add wave -radix hex tx_data
add wave tx_valid tx_ready tx_last
add wave dut/state
add wave -radix unsigned dut/byte_idx

run -all
wave zoom full
