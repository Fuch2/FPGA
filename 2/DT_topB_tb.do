# Запуск тестбенча DT_topB (плата B целиком: опрос, чтение LM75A, индикация, имитатор) в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 DT_topB   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/2; do DT_topB_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv i2c_bit.sv i2c_txn.sv i2c_slave.sv temp_gen.sv dt_model.sv dt_bus.sv dt_reader.sv \
         temp_fmt.sv seg7_dynamic.sv debounce.sv dt_ind.sv DT_topB.sv DT_topB_tb.sv

vsim -voptargs=+acc -onfinish stop DT_topB_tb

add wave -divider "Board"
add wave clk rst_n
add wave -radix binary key led
add wave -divider "External I2C bus"
add wave scl sda
add wave -divider "Polling and source"
add wave dut/poll_p dut/start_read dut/use_model
add wave -radix unsigned dut/gen_mode
add wave -divider "Reader"
add wave dut/u_reader/state dut/busy dut/temp_valid dut/read_fail
add wave -radix hex dut/temp_data
add wave -divider "Bus"
add wave dut/ev_start dut/ev_ack dut/ev_nack dut/ev_stop
add wave dut/done dut/res_nack_addr dut/res_nack_data dut/res_timeout dut/res_arb
add wave dut/u_bus/scl_bus dut/u_bus/sda_bus
add wave -divider "Simulator"
add wave -radix decimal dut/u_bus/u_model/u_gen/temp_val
add wave -divider "Indication"
add wave dut/u_ind/mode dut/u_ind/have_value dut/u_ind/last_ok
add wave -radix hex dut/u_ind/shown_bcd
add wave -radix binary dig seg

run -all
wave zoom full
