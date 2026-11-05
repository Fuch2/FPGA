# Запуск тестбенча eeprom_ring (кольцевой буфер в EEPROM через eeprom_store, i2c_master и eeprom_model)
#   из PowerShell:      .\run_sim.ps1 eeprom_ring   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/2; do eeprom_ring_tb.do
# Большое кольцо - 512 записей, как в PROTOCOL.md; для быстрой проверки можно уменьшить:
#   vsim ... -GBIG_RECORDS=32 eeprom_ring_tb

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv i2c_bit.sv i2c_txn.sv i2c_master.sv i2c_slave.sv eeprom_model.sv \
         eeprom_ctrl.sv eeprom_ring.sv eeprom_store.sv eeprom_ring_tb.sv

vsim -voptargs=+acc -onfinish stop eeprom_ring_tb

add wave clk rst_n
add wave -divider "Ring 8 (env0)"
add wave {ring_cmd_valid[0]} {ring_cmd_ready[0]}
add wave -radix unsigned {ring_cmd_op[0]}
add wave -radix hex {ring_cmd_data[0]} {ring_data[0]}
add wave {ring_done[0]} {ring_ok[0]} {ring_empty[0]} {ring_err[0]} {ring_ovf[0]}
add wave -radix unsigned {ring_count[0]} {ring_wr_ptr[0]} {ring_rd_ptr[0]}
add wave env0/u_store/u_ring/state
add wave env0/u_store/u_ctrl/state
add wave {polling[0]}
add wave -radix hex env0/u_model/ptr

run -all
wave zoom full
