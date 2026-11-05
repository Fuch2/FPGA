# Запуск тестбенча uart_seq_tx в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 uart_seq_tx   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/1; do uart_seq_tx_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv uart_seq_tx.sv uart_seq_tx_tb.sv

vsim -voptargs=+acc -onfinish stop uart_seq_tx_tb

add wave clk rst_n start tx_ready
add wave -divider "String"
add wave dut_str/state
add wave -radix unsigned dut_str/byte_idx
add wave -radix ascii data_s
add wave valid_s last_s busy_s
add wave -divider "One byte"
add wave -radix hex data_1
add wave valid_1 last_1 busy_1

run -all
wave zoom full
