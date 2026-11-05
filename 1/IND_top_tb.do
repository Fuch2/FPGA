# Запуск тестбенча IND_top в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 IND_top   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/1; do IND_top_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv pwm.sv seg7_dynamic.sv debounce.sv test_seq.sv IND_top.sv IND_top_tb.sv

vsim -voptargs=+acc -onfinish stop IND_top_tb

add wave -divider "Board"
add wave clk rst_n
add wave -radix binary key
add wave -radix binary dig seg led
add wave -divider "Inside"
add wave -radix unsigned dut/level dut/duty dut/mode
add wave dut/run dut/pwm_on
add wave -radix hex dut/t_hex
add wave dut/t_valid

run -all
wave zoom full
