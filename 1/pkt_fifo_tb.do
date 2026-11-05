# Запуск тестбенча pkt_fifo в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 pkt_fifo   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/1; do pkt_fifo_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv pkt_fifo.sv pkt_fifo_tb.sv

vsim -voptargs=+acc -onfinish stop pkt_fifo_tb

add wave clk rst_n
add wave -radix hex in_data
add wave in_valid in_ready
add wave -radix hex out_data
add wave out_valid out_ready
add wave -radix unsigned count dut/wr_ptr dut/rd_ptr

run -all
wave zoom full
