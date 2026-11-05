# Запуск тестбенча dt_model (имитатор LM75A: i2c_slave + temp_gen) в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 dt_model   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/2; do dt_model_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv i2c_bit.sv i2c_txn.sv i2c_master.sv temp_gen.sv i2c_slave.sv dt_model.sv dt_model_tb.sv

vsim -voptargs=+acc -onfinish stop dt_model_tb

add wave clk rst_n
add wave -divider "Bus"
add wave scl sda
add wave -divider "Master"
add wave cmd_valid cmd_ready
add wave -radix hex cmd_addr
add wave cmd_read cmd_stop
add wave done res_nack_addr res_nack_data
add wave -radix hex rd_data
add wave rd_valid
add wave -divider "Slave"
add wave dut/u_slave/state
add wave dut/addr_hit dut/addr_read dut/rx_valid dut/tx_req
add wave -radix hex dut/rx_data dut/tx_data
add wave -divider "Registers"
add wave -radix unsigned dut/ptr
add wave -radix hex dut/conf dut/thyst dut/tos dut/temp_snap
add wave -divider "Generator"
add wave dut/tick
add wave -radix unsigned gen_mode
add wave -radix decimal dut/u_gen/temp_val
add wave -radix hex dut/temp_word

run -all
wave zoom full
