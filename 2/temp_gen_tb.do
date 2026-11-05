# Запуск тестбенча temp_gen (генератор последовательности температур) в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 temp_gen   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/2; do temp_gen_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv temp_gen.sv temp_gen_tb.sv

vsim -voptargs=+acc -onfinish stop temp_gen_tb

add wave clk rst_n
add wave tick restart
add wave -radix unsigned gen_mode
add wave -radix decimal gen_lo gen_hi gen_step
add wave -radix hex gen_noise_mask
add wave -radix decimal temp_val
add wave -radix hex temp_word
add wave -radix hex dut/lfsr

run -all
wave zoom full
