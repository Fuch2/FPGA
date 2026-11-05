# Запуск тестбенча связки spi_master + spi_slave в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 spi_link   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/1; do spi_link_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv spi_master.sv spi_slave.sv spi_link_tb.sv

vsim -voptargs=+acc -onfinish stop spi_link_tb

add wave clk rst_n
add wave -divider "SPI"
add wave cs_n sck mosi miso_line
add wave -divider "Master"
add wave -radix hex m_tx_data
add wave m_tx_valid m_tx_ready m_tx_last
add wave -radix hex m_rx_data
add wave m_rx_valid m_busy
add wave u_master/state
add wave -divider "Slave"
add wave -radix hex s_rx_data
add wave s_rx_valid s_frame_start s_frame_end s_frame_short s_tx_underrun
add wave -radix hex s_tx_data
add wave s_tx_valid s_tx_ready s_miso_oe
add wave -radix unsigned u_slave/bit_cnt

run -all
wave zoom full
