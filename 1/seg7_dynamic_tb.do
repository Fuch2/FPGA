# Запуск тестбенча seg7_dynamic в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 seg7_dynamic   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/1; do seg7_dynamic_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv seg7_dynamic.sv seg7_dynamic_tb.sv

vsim -voptargs=+acc -onfinish stop seg7_dynamic_tb

add wave -divider "Inputs"
add wave clk rst_n valid en
add wave -radix hex hex
add wave -radix binary dp blank raw
add wave -radix hex seg_raw
add wave -divider "DUT (active 0)"
add wave -radix unsigned dut_l/idx dut_l/slot_cnt
add wave -radix binary dig_l seg_l
add wave -divider "DUT (active 1)"
add wave -radix binary dig_h seg_h

run -all
wave zoom full
