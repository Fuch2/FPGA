# Тактовый генератор платы 50 МГц
create_clock -name clk -period 20.000 [get_ports clk]
derive_clock_uncertainty

# асинхронные входы (сброс, кнопки, MISO) синхронизируются внутри,
# SPI-выходы и индикация медленные (SCK 1 МГц) - временные проверки не нужны
set_false_path -from [get_ports {rst_n key[*] spi_miso}]
set_false_path -to   [get_ports {spi_cs_n spi_sck spi_mosi seg[*] dig[*] led[*]}]
