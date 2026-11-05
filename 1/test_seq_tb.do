# Запуск тестбенча test_seq в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 test_seq   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/1; do test_seq_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv test_seq.sv test_seq_tb.sv

vsim -voptargs=+acc -onfinish stop test_seq_tb

add wave clk rst_n
add wave -radix unsigned mode
add wave run
add wave -radix hex dut/cnt
add wave -radix unsigned dut/tmr
add wave valid
add wave -radix hex hex
add wave -radix binary dp blank raw
add wave -radix hex seg_raw

run -all
wave zoom full
