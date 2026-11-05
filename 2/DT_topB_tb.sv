// =============================================================================
// Тестбенч для DT_topB (плата B целиком): таймер опроса, кнопки -> dt_reader -> dt_bus
// (мастер + выбор шины + имитатор) -> dt_ind (индикатор и светодиоды)
//
// «Реальный датчик» на выводах шины - тоже dt_model (экземпляр sensor), его температурой
// управляет тестбенч. Параметры уменьшены: антидребезг 20 тактов, слот индикатора
// 20 тактов, вспышка 30 тактов, опрос раз в 4 мс, обновление имитатора внутри - 1 мс.
// Шина настоящая: 100 кГц. Картинку индикатора тестбенч «читает» с выводов seg / dig
// (как глаз). Монитор линий I2CLineMon следит за временами на внешней шине.
// Проверки:
//   1. После сброса первый опрос выполняется сам (температура ещё 0.0), второй через 4 мс:
//      индикатор « 25.4» (25.375 °C), были START и ACK, NACK не было
//   2. KEY1: чтение по запросу: «-5.3», «-25.4», «125.0», « 0.0»
//   3. Датчик отключён: « Err», led[3] горит, был NACK; после включения температура
//      возвращается, led[3] остаётся до смены источника
//   4. KEY3 по кругу: режим hex «7D00» (125.0 °C), режимы счётчиков и источника
//   5. KEY2: переключение на имитатор: led[3] гаснет, индикатор «5 1 . 2» (источник, режим),
//      внешняя шина больше не используется (нет START); температура пилы 20.0 .. 30.0 °C,
//      значения меняются
//   6. KEY4: режимы имитатора: шум (20.0 .. 21.9), константа 20.0, линейный (стоит на 30.0)
//   7. KEY2 обратно: датчик на выводах снова читается, внешняя шина оживает
//   8. Монитор линий: нет нарушений времён на внешней шине
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps
`include "i2c_line_mon.svh"

module DT_topB_tb;

    localparam int  CLK_FREQ = 50_000_000;
    localparam real CLK_NS   = 1.0e9 / CLK_FREQ;
    localparam int  SLOT     = 20;
    localparam int  BLANK    = 2;
    localparam int  FLASH    = 30;
    localparam int  DEB      = 20;

    localparam logic [6:0] SEG_REF [16] = '{
        7'h3F, 7'h06, 7'h5B, 7'h4F, 7'h66, 7'h6D, 7'h7D, 7'h07,
        7'h7F, 7'h6F, 7'h77, 7'h7C, 7'h39, 7'h5E, 7'h79, 7'h71
    };

    class Check;
        int errors = 0;

        function void err(string msg);
            errors++;
            if (errors <= 20) $display("[%0t] ERROR: %s", $time, msg);
        endfunction
    endclass

    // -------------------------------------------------------------------------
    // Сигналы и DUT
    // -------------------------------------------------------------------------
    logic       clk   = 1'b0;
    logic       rst_n = 1'b0;
    logic [3:0] key   = 4'b1111;
    logic [7:0] seg;
    logic [3:0] dig, led;
    tri1        scl, sda;

    // «реальный датчик» на выводах шины
    logic               sensor_present = 1'b1, sensor_restart = 1'b0;
    logic signed [10:0] sensor_lo = 11'sd203;
    logic               sensor_sda_low;

    always #(CLK_NS / 2) clk = ~clk;

    DT_topB #(
        .CLK_FREQ        (CLK_FREQ),
        .POLL_MS         (4),
        .MODEL_UPDATE_MS (1),
        .DEB_CLKS        (DEB),
        .DIGIT_HZ        (CLK_FREQ / SLOT),
        .BLANK_CLKS      (BLANK),
        .FLASH_CLKS      (FLASH)
    ) dut (
        .clk (clk), .rst_n (rst_n), .key (key),
        .i2c_scl (scl), .i2c_sda (sda),
        .seg (seg), .dig (dig), .led (led)
    );

    dt_model #(
        .CLK_FREQ  (CLK_FREQ),
        .ADDR      (7'h48),
        .UPDATE_MS (1)
    ) sensor (
        .clk (clk), .rst_n (rst_n),
        .present (sensor_present), .gen_restart (sensor_restart), .gen_mode (2'd0),
        .gen_lo (sensor_lo), .gen_hi (11'sd1000), .gen_step (11'd8), .gen_noise_mask (8'h00),
        .scl_in (scl), .sda_in (sda), .sda_low (sensor_sda_low)
    );
    assign sda = sensor_sda_low ? 1'b0 : 1'bz;

    Check      chk = new();
    I2CLineMon mon = new();

    always @(posedge scl) if (rst_n) mon.scl_rise($realtime, sda);
    always @(negedge scl) if (rst_n) mon.scl_fall($realtime, sda);
    always @(posedge sda) if (rst_n) mon.sda_rise($realtime, scl);
    always @(negedge sda) if (rst_n) mon.sda_fall($realtime, scl);

    wire [7:0] seg_on = ~seg;
    wire [3:0] dig_on = ~dig;
    wire [3:0] led_on = ~led;

    logic [3:0] led_seen = '0;
    always @(posedge clk) led_seen <= led_seen | led_on;

    // -------------------------------------------------------------------------
    // Кнопки, ожидание
    // -------------------------------------------------------------------------
    task automatic press(input int k, input int hold = 3 * DEB);
        key[k] = 1'b0;
        repeat (hold) @(posedge clk);
        key[k] = 1'b1;
        repeat (DEB + 5) @(posedge clk);
    endtask

    task automatic wait_ms(input int ms);
        #(ms * 1_000_000.0);
    endtask

    // -------------------------------------------------------------------------
    // Индикатор: картинка с выводов
    // -------------------------------------------------------------------------
    string HEX_CHARS = "0123456789ABCDEF";

    function automatic string seg_char(logic [7:0] f);
        string s;
        s = "?";
        if (f[6:0] === 7'h00) s = " ";
        else if (f[6:0] === 7'h40) s = "-";
        else if (f[6:0] === 7'h50) s = "r";
        else begin
            for (int i = 0; i < 16; i++)
                if (f[6:0] === SEG_REF[i]) s = HEX_CHARS.substr(i, i);
        end
        if (f[7]) s = {s, "."};
        return s;
    endfunction

    task automatic read_text(output string text);
        logic [7:0] f[4];
        foreach (f[i]) f[i] = '0;
        repeat (2 * 4 * SLOT) begin
            @(posedge clk);
            for (int i = 0; i < 4; i++)
                if (dig_on[i]) f[i] = seg_on;
        end
        text = {seg_char(f[3]), seg_char(f[2]), seg_char(f[1]), seg_char(f[0])};
    endtask

    task automatic goto_mode(input int m);
        repeat (6) begin
            if (int'(dut.u_ind.mode) == m) return;
            press(2);
        end
        chk.err($sformatf("display mode %0d is not reachable", m));
    endtask

    task automatic expect_text(input string name, input string want);
        string t;
        read_text(t);
        if (t != want) chk.err($sformatf("%s: display \"%s\", expected \"%s\"", name, t, want));
        else           $display("[%0t]    %s: \"%s\"", $time, name, t);
    endtask

    // температура с индикатора в градусах (режим «температура»)
    task automatic read_degrees(output real deg, output string t);
        string clean;
        read_text(t);
        clean = "";
        for (int i = 0; i < t.len(); i++)
            if (t[i] != " ") clean = {clean, t.substr(i, i)};
        deg = 1.0e9;
        void'($sscanf(clean, "%f", deg));
    endtask

    // датчик на выводах выдаёт температуру raw (шаги 0.125 °C)
    task automatic set_sensor(input int raw);
        @(negedge clk);
        sensor_lo = raw;
        @(negedge clk);
        sensor_restart = 1'b1;
        @(negedge clk);
        sensor_restart = 1'b0;
        wait_ms(2);                                 // температура обновилась, прошёл хотя бы один опрос
    endtask

    // чтение по запросу и ожидание результата
    task automatic read_now();
        press(0);
        wait_ms(3);
    endtask

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        string t;
        real   deg, prev;
        int    starts0, n_diff;
        $timeformat(-6, 2, " us", 0);
        repeat (3) @(posedge clk);
        rst_n = 1'b1;

        $display("[%0t] 1. First poll after reset", $time);
        repeat (10) @(posedge clk);
        if (led_on[3] !== 1'b0) chk.err("led[3] is on after reset");
        wait_ms(2);
        expect_text("first poll (sensor not updated yet)", "  0.0");  // модель датчика обновится только через 1 мс
        wait_ms(4);                                                   // второй опрос
        expect_text("25.375 degC", " 25.4");
        if (led_seen[0] !== 1'b1 || led_seen[1] !== 1'b1 || led_seen[2] !== 1'b0 || led_seen[3] !== 1'b0)
            chk.err($sformatf("leds seen = %b, expected 0011", led_seen));

        $display("[%0t] 2. KEY1: read on request", $time);
        sensor_lo = -42;  sensor_restart = 1'b1;  @(posedge clk);  sensor_restart = 1'b0;
        wait_ms(2);
        read_now();
        expect_text("-5.25 degC", " -5.3");
        set_sensor(-203);
        read_now();
        expect_text("-25.375 degC", "-25.4");
        set_sensor(1000);
        read_now();
        expect_text("125.0 degC", "125.0");
        set_sensor(0);
        read_now();
        expect_text("0.0 degC", "  0.0");
        set_sensor(1000);
        read_now();

        $display("[%0t] 3. Sensor disconnected", $time);
        led_seen = '0;
        sensor_present = 1'b0;
        read_now();
        expect_text("sensor off", " Err");
        if (led_on[3] !== 1'b1) chk.err("led[3] is off after a failed read");
        if (led_seen[2] !== 1'b1) chk.err("led[2] (NACK) never lit");
        sensor_present = 1'b1;
        read_now();
        expect_text("sensor back", "125.0");
        if (led_on[3] !== 1'b1) chk.err("led[3] went off before KEY2");

        $display("[%0t] 4. KEY3 modes", $time);
        goto_mode(1);
        expect_text("hex", "7D00");
        goto_mode(2);
        read_text(t);
        if (t[0] != "r") chk.err($sformatf("r mode shows \"%s\"", t));
        goto_mode(3);
        read_text(t);
        if (t[0] != "E") chk.err($sformatf("E mode shows \"%s\"", t));
        goto_mode(4);
        expect_text("source (sensor)", "50 2");

        $display("[%0t] 5. KEY2: simulator instead of the sensor", $time);
        starts0 = mon.n_start;
        press(1);
        if (dut.use_model !== 1'b1) chk.err("KEY2 did not switch to the simulator");
        repeat (5) @(posedge clk);
        if (led_on[3] !== 1'b0) chk.err("led[3] is still on after KEY2");
        expect_text("source (simulator)", "51 2");
        goto_mode(0);
        prev = -1000.0;
        n_diff = 0;
        wait_ms(5);
        for (int i = 0; i < 10; i++) begin
            wait_ms(3);
            read_degrees(deg, t);
            if (deg < 19.99 || deg > 30.01) chk.err($sformatf("saw: \"%s\" is outside 20.0 .. 30.0", t));
            if (deg != prev) n_diff++;
            prev = deg;
        end
        if (n_diff < 4) chk.err($sformatf("saw: temperature changed only %0d times", n_diff));
        if (mon.n_start != starts0) chk.err("the external bus was used while the simulator is selected");
        if (scl !== 1'b1 || sda !== 1'b1) chk.err("external bus pins are not released");

        $display("[%0t] 6. KEY4: simulator modes", $time);
        press(3);                                           // 3 - шум
        expect_text("source (noise)", "51 3");
        goto_mode(0);
        n_diff = 0;
        prev = -1000.0;
        for (int i = 0; i < 10; i++) begin
            wait_ms(3);
            read_degrees(deg, t);
            if (deg < 19.99 || deg > 21.91) chk.err($sformatf("noise: \"%s\" is outside 20.0 .. 21.9", t));
            if (deg != prev) n_diff++;
            prev = deg;
        end
        if (n_diff < 3) chk.err($sformatf("noise: temperature changed only %0d times", n_diff));
        press(3);                                           // 0 - константа
        goto_mode(0);
        wait_ms(5);
        expect_text("constant", " 20.0");
        press(3);                                           // 1 - линейный рост до 30.0 и остановка
        goto_mode(0);
        wait_ms(35);
        expect_text("linear end", " 30.0");
        wait_ms(8);
        expect_text("linear stays", " 30.0");

        $display("[%0t] 7. KEY2: back to the sensor", $time);
        starts0 = mon.n_start;
        press(1);
        if (dut.use_model !== 1'b0) chk.err("KEY2 did not switch back to the sensor");
        goto_mode(0);
        wait_ms(6);
        expect_text("sensor again", "125.0");
        if (mon.n_start == starts0) chk.err("the external bus is not used after switching back");

        $display("[%0t] 8. Line monitor", $time);
        if (mon.errors != 0) chk.err($sformatf("line monitor: %0d violations", mon.errors));

        $display("------------------------------------------------------------");
        $display("ERRORS: %0d", chk.errors);
        if (chk.errors == 0) $display("TEST PASSED");
        else                 $display("TEST FAILED");
        $display("------------------------------------------------------------");
        $finish;
    end

    initial begin
        #400ms;
        $display("TEST FAILED: global timeout");
        $finish;
    end

endmodule
