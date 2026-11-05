# Запуск тестбенча I2C_topB (плата B целиком + модели «датчика» и «EEPROM») в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 I2C_topB   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/2; do I2C_topB_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv i2c_bit.sv i2c_txn.sv i2c_master.sv i2c_seq.sv i2c_res_parser.sv \
         seg7_dynamic.sv debounce.sv i2c_ind.sv i2c_slave_model.sv \
         I2C_topB.sv I2C_topB_tb.sv

vsim -voptargs=+acc -onfinish stop I2C_topB_tb

add wave -divider "Board"
add wave clk rst_n
add wave -radix binary key led
add wave -divider "I2C bus"
add wave scl sda
add wave -divider "Sequencer"
add wave -radix unsigned dut/script
add wave dut/u_seq/state dut/u_seq/busy dut/u_seq/scan_active
add wave -radix unsigned dut/u_seq/step_n
add wave -divider "Master"
add wave dut/cmd_valid dut/cmd_ready
add wave -radix hex dut/cmd_addr
add wave dut/cmd_read dut/cmd_stop
add wave dut/done dut/res_nack_addr dut/res_nack_data dut/res_timeout dut/res_arb
add wave dut/ev_start dut/ev_ack dut/ev_nack dut/ev_stop
add wave dut/u_i2c/u_txn/state
add wave -divider "Parser"
add wave -radix hex dut/rd_word dut/last_addr dut/last_status
add wave -radix unsigned dut/found_cnt
add wave -divider "Indication"
add wave dut/u_ind/mode
add wave -radix unsigned dut/u_ind/found_idx
add wave -radix hex dut/u_ind/cnt_txn dut/u_ind/cnt_nack dut/u_ind/cnt_err

run -all
wave zoom full
