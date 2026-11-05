# Запуск тестбенча i2c_ind в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 i2c_ind   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/2; do i2c_ind_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv seg7_dynamic.sv i2c_ind.sv i2c_ind_tb.sv

vsim -voptargs=+acc -onfinish stop i2c_ind_tb

add wave clk rst_n
add wave -divider "Events"
add wave ev_start ev_ack ev_nack txn_p nack_p err_p
add wave -divider "Data"
add wave -radix hex info_word info_addr info_status
add wave -radix unsigned found_cnt found_idx script
add wave -radix hex found_addr
add wave -divider "Control"
add wave run run_scan script_changed clear next_mode
add wave -divider "Outputs"
add wave -radix binary dig led
add wave -radix hex seg
add wave -divider "Inside"
add wave dut/mode
add wave -radix hex dut/cnt_txn dut/cnt_nack dut/cnt_err
add wave dut/err_seen

run -all
wave zoom full
