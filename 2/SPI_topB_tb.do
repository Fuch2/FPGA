# Запуск тестбенча SPI_topB в ModelSim / Questa
#   из PowerShell:      .\run_sim.ps1 SPI_topB   (или с ключом -Gui)
#   в консоли ModelSim: cd C:/Users/aynur/Documents/FPGA/2; do SPI_topB_tb.do

if {[file exists work]} { vdel -lib work -all }
vlib work
vlog -sv spi_master.sv spi_slave.sv \
         spi_cmd_parser.sv spi_resp_tx.sv spi_mi_stub.sv \
         seg7_dynamic.sv debounce.sv spi_ind.sv \
         SPI_topB.sv SPI_topB_tb.sv

vsim -voptargs=+acc -onfinish stop SPI_topB_tb

add wave -divider "Board"
add wave clk rst_n
add wave -radix binary key led
add wave -divider "SPI"
add wave cs_n sck mosi miso_line
add wave -divider "Slave"
add wave dut/frame_start dut/frame_end dut/frame_short
add wave -radix hex dut/s_rx_data dut/s_tx_data
add wave dut/s_rx_valid dut/s_tx_valid dut/s_tx_ready dut/s_tx_underrun
add wave -divider "Commands"
add wave dut/cmd_start dut/cmd_measure dut/cmd_stop dut/cmd_nop dut/cmd_unknown dut/frame_err
add wave -divider "MI stub"
add wave -radix hex dut/mi_status dut/mi_data
add wave dut/mi_running dut/meas_done
add wave -divider "Indication"
add wave dut/u_ind/mode
add wave -radix hex dut/u_ind/cnt_tx dut/u_ind/cnt_rx dut/u_ind/cnt_err

run -all
wave zoom full
