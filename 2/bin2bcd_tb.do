# Запуск тестбенча bin2bcd (двоичное число -> десятичные цифры)
#   из PowerShell:      .\run_sim.ps1 bin2bcd   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/2; do bin2bcd_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv bin2bcd.sv bin2bcd_tb.sv

vsim -voptargs=+acc -onfinish stop bin2bcd_tb

add wave clk rst_n
add wave start busy
add wave -radix unsigned bin
add wave -radix hex bcd
add wave -radix unsigned dut/iter

run -all
wave zoom full
