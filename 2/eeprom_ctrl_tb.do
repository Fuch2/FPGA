# Запуск тестбенча eeprom_ctrl (запись с ACK-polling, чтение; имитатор eeprom_model и i2c_slave_model)
#   из PowerShell:      .\run_sim.ps1 eeprom_ctrl   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/2; do eeprom_ctrl_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv i2c_bit.sv i2c_txn.sv i2c_master.sv i2c_slave.sv i2c_slave_model.sv eeprom_model.sv \
         eeprom_ctrl.sv eeprom_ctrl_tb.sv

vsim -voptargs=+acc -onfinish stop eeprom_ctrl_tb

add wave clk rst_n
add wave -divider "Env 0 (i2c_slave_model)"
add wave {scl[0]} {sda[0]}
add wave {req_valid[0]} {req_ready[0]}
add wave -radix unsigned {req_op[0]}
add wave -radix hex {req_addr[0]} {req_data[0]}
add wave env0/u_ctrl/state
add wave {data_valid[0]} {data_ready[0]} {data_last[0]}
add wave -radix hex {data[0]}
add wave {op_done[0]} {op_ok[0]} {op_nack[0]} {op_bus[0]} {op_poll[0]} {polling[0]}
add wave -radix unsigned {op_polls[0]}
add wave -divider "Env 1 (eeprom_model)"
add wave env1/u_ctrl/state
add wave {op_done[1]} {op_ok[1]}
add wave model1_busy
add wave -radix unsigned model1_cycles
add wave -radix hex model1/ptr

run -all
wave zoom full
