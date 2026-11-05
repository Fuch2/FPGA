# Запуск тестбенча SPI_topA в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 SPI_topA   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/1; do SPI_topA_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv spi_master.sv spi_slave.sv spi_cmd_tx.sv \
         seg7_dynamic.sv debounce.sv spi_ind.sv \
         SPI_topA.sv SPI_topA_tb.sv

vsim -voptargs=+acc -onfinish stop SPI_topA_tb

add wave -divider "Board"
add wave clk rst_n
add wave -radix binary key led
add wave -divider "SPI"
add wave spi_cs_n spi_sck spi_mosi miso_line
add wave -divider "Pipeline"
add wave dut/u_cmd_tx/state
add wave dut/u_cmd_tx/busy dut/req_lost
add wave dut/rsp_valid dut/rsp_final dut/rsp_ok
add wave -radix hex dut/rsp_cmd dut/rsp_status dut/rsp_data
add wave -divider "Slave model"
add wave s_frame_start s_frame_end
add wave -radix hex s_rx_data s_tx_data
add wave -divider "Indication"
add wave dut/u_ind/mode
add wave -radix hex dut/u_ind/cnt_tx dut/u_ind/cnt_rx dut/u_ind/cnt_err

run -all
wave zoom full
