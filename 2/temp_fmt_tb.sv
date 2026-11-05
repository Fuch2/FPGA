// =============================================================================
// Тестбенч для temp_fmt: температура Q8.8 -> десятичные цифры (десятые доли градуса)
//
// Класс RefFmt - эталон в целых числах: знак, модуль в шагах 0.125 °C, десятые
// ((|T|*5 + 2) / 4), ограничение 99.9 для отрицательных, разложение на цифры.
// Проверки:
//   1. После сброса out_valid = 0, busy = 0
//   2. Примеры: 25.375 -> 0254, 0 -> 0000, -1.0 -> neg 0010, +125 -> 1250, -55 -> neg 0550,
//      -0.125 -> neg 0001, +127.875 -> 1279, -128 -> neg 0999 (ограничение)
//   3. Все 2048 значений 11 бит: результат равен эталону; младшие 5 бит входа
//      (мусор датчика) на результат не влияют
//   4. out_valid - импульс на 1 такт ровно через CONV_LATENCY тактов после запроса;
//      busy = 1 между запросом и результатом
//   5. Запрос во время преобразования игнорируется (результат первого запроса)
//   6. Запросы подряд с минимальным интервалом
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module temp_fmt_tb;

    localparam real CLK_NS       = 20.0;
    localparam int  CONV_LATENCY = 12;      // тактов от запроса до out_valid

    class RefFmt;
        bit          neg;
        logic [15:0] bcd;

        function void calc(logic [15:0] t);
            int raw, mag, tenths;
            raw = $signed(t[15:5]);
            neg = (raw < 0);
            mag = neg ? -raw : raw;
            tenths = (mag * 5 + 2) / 4;
            if (neg && tenths > 999) tenths = 999;
            bcd = {4'(tenths / 1000), 4'((tenths / 100) % 10), 4'((tenths / 10) % 10), 4'(tenths % 10)};
        endfunction
    endclass

    logic        clk = 1'b0, rst_n = 1'b0;
    logic        in_valid = 1'b0;
    logic [15:0] temp_data = '0;
    logic        busy, out_valid, neg;
    logic [15:0] bcd;

    int errors = 0;
    int n_out  = 0;

    always #(CLK_NS / 2) clk = ~clk;

    temp_fmt dut (.*);

    RefFmt ref_f = new();

    always @(posedge clk) if (out_valid) n_out++;

    task automatic err(string msg);
        errors++;
        if (errors <= 20) $display("[%0t] ERROR: %s", $time, msg);
    endtask

    // запрос и ожидание результата; latency - тактов от запроса до out_valid
    task automatic convert(input logic [15:0] t, output int latency);
        @(negedge clk);
        temp_data = t;
        in_valid  = 1'b1;
        @(negedge clk);
        in_valid  = 1'b0;
        latency = 1;
        while (!out_valid && latency < 100) begin
            @(negedge clk);
            latency++;
        end
    endtask

    task automatic check_value(input string name, input logic [15:0] t);
        int lat;
        convert(t, lat);
        ref_f.calc(t);
        if (neg !== ref_f.neg || bcd !== ref_f.bcd)
            err($sformatf("%s: T=%h neg=%b bcd=%h, expected neg=%b bcd=%h", name, t, neg, bcd, ref_f.neg, ref_f.bcd));
        if (lat != CONV_LATENCY) err($sformatf("%s: latency %0d, expected %0d", name, lat, CONV_LATENCY));
        @(negedge clk);
        if (busy || out_valid) err($sformatf("%s: busy / out_valid after the result", name));
    endtask

    task automatic check_example(input string name, input logic [15:0] t, input bit e_neg, input logic [15:0] e_bcd);
        int lat;
        convert(t, lat);
        if (neg !== e_neg || bcd !== e_bcd)
            err($sformatf("%s: neg=%b bcd=%h, expected neg=%b bcd=%h", name, neg, bcd, e_neg, e_bcd));
        @(negedge clk);
    endtask

    initial begin
        int lat;
        #(CLK_NS * 3);
        if (out_valid !== 1'b0 || busy !== 1'b0) err("after reset out_valid / busy are not 0");
        rst_n = 1'b1;
        repeat (3) @(posedge clk);

        // 2. примеры
        check_example("25.375", 16'h1960, 0, 16'h0254);
        check_example("0",      16'h0000, 0, 16'h0000);
        check_example("-1.0",   16'hFF00, 1, 16'h0010);
        check_example("+125",   16'h7D00, 0, 16'h1250);
        check_example("-55",    {11'(-440), 5'b0}, 1, 16'h0550);
        check_example("-0.125", 16'hFFE0, 1, 16'h0001);
        check_example("+127.875", 16'h7FE0, 0, 16'h1279);
        check_example("-128",   16'h8000, 1, 16'h0999);

        // 3. все значения датчика, низкие 5 бит - мусор
        for (int r = -1024; r < 1024; r++) begin
            logic [15:0] t;
            t = {11'(r), 5'($urandom)};
            check_value($sformatf("raw %0d", r), t);
        end

        // 5. запрос во время преобразования
        @(negedge clk);
        temp_data = 16'h1960;  in_valid = 1'b1;
        @(negedge clk);
        temp_data = 16'hFF00;                                    // второй запрос, in_valid остаётся 1
        repeat (5) @(negedge clk);
        in_valid = 1'b0;
        if (!busy) err("busy = 0 during conversion");
        while (!out_valid) @(negedge clk);
        if (neg !== 1'b0 || bcd !== 16'h0254) err($sformatf("request during busy: neg=%b bcd=%h, expected the first (0254)", neg, bcd));
        repeat (4) @(negedge clk);

        // 6. запросы подряд
        n_out = 0;
        for (int i = 0; i < 20; i++) begin
            convert($urandom, lat);
            ref_f.calc(temp_data);
            if (neg !== ref_f.neg || bcd !== ref_f.bcd) err($sformatf("back-to-back %0d: bcd=%h expected %h", i, bcd, ref_f.bcd));
        end
        repeat (2) @(negedge clk);
        if (n_out != 20) err($sformatf("%0d out_valid pulses for 20 requests", n_out));

        repeat (5) @(posedge clk);
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED: %0d errors", errors);
        $finish;
    end

endmodule
