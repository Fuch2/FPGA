# Запуск тестбенча temp_fmt (Q8.8 -> десятичные цифры) в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 temp_fmt   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/2; do temp_fmt_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv temp_fmt.sv temp_fmt_tb.sv

vsim -voptargs=+acc -onfinish stop temp_fmt_tb

add wave clk rst_n
add wave in_valid
add wave -radix hex temp_data
add wave dut/state
add wave -radix unsigned dut/cnt
add wave -radix hex dut/acc dut/bin
add wave busy out_valid neg
add wave -radix hex bcd

run -all
wave zoom full
