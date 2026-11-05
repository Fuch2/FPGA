# Запуск тестбенча dt_ind (индикация чтения ДТ) в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 dt_ind   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/2; do dt_ind_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv seg7_dynamic.sv temp_fmt.sv dt_ind.sv dt_ind_tb.sv

vsim -voptargs=+acc -onfinish stop dt_ind_tb

add wave clk rst_n
add wave -divider "Inputs"
add wave ev_start ev_ack ev_nack temp_valid read_fail
add wave -radix hex temp_data
add wave clear next_mode show_temp show_src
add wave -divider "Inside"
add wave dut/mode
add wave dut/have_value dut/last_ok
add wave -radix hex dut/shown_raw dut/shown_bcd dut/cnt_ok dut/cnt_fail
add wave -divider "Display"
add wave -radix binary dig seg led

run -all
wave zoom full
