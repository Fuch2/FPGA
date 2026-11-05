// =============================================================================
// Монитор линий I2C для тестбенчей (включается через `include)
//
// Класс I2CLineMon проверяет по фронтам SCL и SDA временные требования
// Standard-mode (100 кГц) и считает START / STOP на шине. Тестбенч вызывает
// scl_rise / scl_fall / sda_rise / sda_fall из блоков always @(posedge/negedge ...)
// с текущим временем и уровнем другой линии.
//   tLOW  >= 4.7 мкс (SCL = 0 между спадом и фронтом)
//   tHIGH >= 4.0 мкс (SCL = 1 между фронтом и спадом)
//   tSU;STA >= 4.7 мкс (фронт SCL -> спад SDA при повторном START)
//   tHD;STA >= 4.0 мкс (спад SDA -> спад SCL)
//   tSU;STO >= 4.0 мкс (фронт SCL -> фронт SDA при STOP)
//   tBUF   >= 4.7 мкс (STOP -> START)
//   tSU;DAT >= 250 нс (изменение SDA -> фронт SCL)
//   tHD;DAT >= 0 (SDA не меняется раньше спада SCL)
//   период SCL (фронт -> фронт) не короче MIN_PERIOD_NS (если check_period = 1)
// enable = 0 отключает проверки (на время искусственных неисправностей).
// =============================================================================
class I2CLineMon;
    real MIN_LOW_NS   = 4700.0;
    real MIN_HIGH_NS  = 4000.0;
    real MIN_SU_STA   = 4700.0;
    real MIN_HD_STA   = 4000.0;
    real MIN_SU_STO   = 4000.0;
    real MIN_BUF      = 4700.0;
    real MIN_SU_DAT   = 250.0;
    real MIN_PERIOD   = 9900.0;     // не быстрее ~101 кГц
    real MAX_PERIOD   = 10300.0;    // не медленнее ~97 кГц (когда нет растяжки)

    bit  enable       = 1;
    bit  check_period = 1;

    int  n_start = 0, n_stop = 0, n_scl_rise = 0, errors = 0;

    real t_scl_rise = -1.0e9, t_scl_fall = -1.0e9, t_sda_chg = -1.0e9;
    real t_start = -1.0e9, t_stop = -1.0e9;
    real t_data_rise = -1.0e9;      // прошлый фронт SCL данных (после START / STOP сбрасывается)
    bit  stopped = 1;

    function void err(string msg);
        errors++;
        if (errors <= 20) $display("[%0t] BUS ERROR: %s", $time, msg);
    endfunction

    function void need(string name, real t_now, real t_prev, real min_ns);
        if (enable && t_now - t_prev < min_ns)
            err($sformatf("%s = %0.0f ns, need >= %0.0f", name, t_now - t_prev, min_ns));
    endfunction

    function void scl_rise(real t, bit sda);
        need("tLOW", t, t_scl_fall, MIN_LOW_NS);
        need("tSU;DAT", t, t_sda_chg, MIN_SU_DAT);
        if (check_period && enable && t - t_data_rise < 20000.0) begin
            if (t - t_data_rise < MIN_PERIOD)
                err($sformatf("SCL period = %0.0f ns, need >= %0.0f", t - t_data_rise, MIN_PERIOD));
            if (t - t_data_rise > MAX_PERIOD && t - t_scl_fall < 6000.0)
                err($sformatf("SCL period = %0.0f ns, need <= %0.0f", t - t_data_rise, MAX_PERIOD));
        end
        t_data_rise = t;
        n_scl_rise++;
        t_scl_rise = t;
    endfunction

    function void scl_fall(real t, bit sda);
        need("tHIGH", t, t_scl_rise, MIN_HIGH_NS);
        if (t - t_start < 100000.0 && t_start > t_scl_fall) need("tHD;STA", t, t_start, MIN_HD_STA);
        if (enable && t - t_sda_chg < 0.0) err("SDA changed before SCL fall");
        t_scl_fall = t;
    endfunction

    // SDA вырос при SCL = 1 - STOP; при SCL = 0 - обычное изменение данных
    function void sda_rise(real t, bit scl);
        if (scl) begin
            need("tSU;STO", t, t_scl_rise, MIN_SU_STO);
            n_stop++;
            t_data_rise = -1.0e9;
            t_stop  = t;
            stopped = 1;
        end else begin
            need("tHD;DAT", t, t_scl_fall, 0.0);
        end
        t_sda_chg = t;
    endfunction

    // SDA упал при SCL = 1 - START (или повторный START)
    function void sda_fall(real t, bit scl);
        if (scl) begin
            if (stopped) need("tBUF", t, t_stop, MIN_BUF);
            else         need("tSU;STA", t, t_scl_rise, MIN_SU_STA);
            n_start++;
            t_data_rise = -1.0e9;
            t_start = t;
            stopped = 0;
        end else begin
            need("tHD;DAT", t, t_scl_fall, 0.0);
        end
        t_sda_chg = t;
    endfunction
endclass
