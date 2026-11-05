# Запуск тестбенча debounce в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 debounce   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/1; do debounce_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv debounce.sv debounce_tb.sv

vsim -voptargs=+acc -onfinish stop debounce_tb

add wave clk rst_n btn
add wave -radix unsigned dut_l/cnt
add wave pressed_l press_l release_l
add wave pressed_h press_h release_h

run -all
wave zoom full
