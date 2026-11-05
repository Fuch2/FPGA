# Тактовый генератор платы 50 МГц
create_clock -name clk -period 20.000 [get_ports clk]
derive_clock_uncertainty

# асинхронные входы (сброс, кнопки, SCL и SDA) синхронизируются внутри
# (i2c_bit и i2c_slave: триггеры на SCL и на SDA), выходы медленные
set_false_path -from [get_ports {rst_n key[*] i2c_scl i2c_sda}]
set_false_path -to   [get_ports {i2c_scl i2c_sda seg[*] dig[*] led[*]}]
