# Запуск тестбенча spi_cmd_tx в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 spi_cmd_tx   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/1; do spi_cmd_tx_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv spi_master.sv spi_slave.sv spi_cmd_tx.sv spi_cmd_tx_tb.sv

vsim -voptargs=+acc -onfinish stop spi_cmd_tx_tb

add wave clk rst_n
add wave -divider "Requests"
add wave req_start req_measure req_stop busy req_lost
add wave -divider "Results"
add wave rsp_valid rsp_final rsp_ok
add wave -radix hex rsp_cmd rsp_status rsp_data
add wave -divider "Pipeline"
add wave dut/state
add wave -radix hex m_tx_data
add wave m_tx_valid m_tx_ready m_tx_last m_rx_valid
add wave -divider "SPI"
add wave cs_n sck mosi miso_line
add wave -divider "Slave"
add wave s_frame_start s_frame_end
add wave -radix hex s_rx_data s_tx_data

run -all
wave zoom full
