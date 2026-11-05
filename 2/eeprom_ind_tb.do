# Запуск тестбенча eeprom_ind (индикация проверки EEPROM: адрес, данные, указатели, светодиоды)
#   из PowerShell:      .\run_sim.ps1 eeprom_ind   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/2; do eeprom_ind_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv bin2bcd.sv seg7_dynamic.sv eeprom_ind.sv eeprom_ind_tb.sv

vsim -voptargs=+acc -onfinish stop eeprom_ind_tb

add wave clk rst_n
add wave -divider "Inputs"
add wave -radix hex info_data info_addr cursor ring_wr_ptr ring_rd_ptr
add wave -radix unsigned ring_count polls_last
add wave ring_ovf info_empty wr_p rd_p err_p polling next_mode run clear
add wave -divider "Inside"
add wave dut/mode dut/screen dut/auto_data
add wave -radix hex dut/cnt_err dut/count_bcd dut/polls_bcd
add wave -divider "Outputs"
add wave -radix hex seg
add wave dig led

run -all
wave zoom full
