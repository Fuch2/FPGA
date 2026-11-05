# Запуск тестбенча eeprom_seq (сценарии проверки EEPROM: кольцо, прямая запись и чтение)
#   из PowerShell:      .\run_sim.ps1 eeprom_seq   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/2; do eeprom_seq_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv i2c_bit.sv i2c_txn.sv i2c_slave.sv eeprom_model.sv eeprom_bus.sv \
         eeprom_ctrl.sv eeprom_ring.sv eeprom_store.sv eeprom_seq.sv eeprom_seq_tb.sv

vsim -voptargs=+acc -onfinish stop eeprom_seq_tb

add wave clk rst_n
add wave -divider "Sequencer"
add wave run
add wave -radix unsigned script
add wave dut/state busy finished error
add wave -radix hex info_addr info_data cursor
add wave info_empty wr_p rd_p err_p polling
add wave -radix unsigned polls_last
add wave -divider "Ring"
add wave -radix unsigned ring_count ring_wr_ptr ring_rd_ptr
add wave ring_ovf ring_busy
add wave -divider "Model"
add wave model_busy
add wave -radix unsigned model_cycles

run -all
wave zoom full
