# Тактовый генератор платы 50 МГц
create_clock -name clk -period 20.000 [get_ports clk]
derive_clock_uncertainty

# асинхронные входы (сброс, кнопки, CS_N, SCK, MOSI) синхронизируются внутри
# (spi_slave: два триггера на каждый вход), MISO и индикация медленные
set_false_path -from [get_ports {rst_n key[*] spi_cs_n spi_sck spi_mosi}]
set_false_path -to   [get_ports {spi_miso seg[*] dig[*] led[*]}]
