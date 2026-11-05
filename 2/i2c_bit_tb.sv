// =============================================================================
// Тестбенч для i2c_bit (нижний уровень I2C: START, STOP, бит), 50 МГц, SCL 100 кГц
//
// Ведомого нет: линиями «открытого стока» управляют DUT и сам тестбенч (он же
// имитирует растяжку SCL и неисправности). Монитор линий (i2c_line_mon.svh,
// класс I2CLineMon) по фронтам на проводах проверяет tLOW, tHIGH, tSU;STA, tHD;STA,
// tSU;STO, tBUF, tSU;DAT, период SCL; считает START и STOP.
// Классы:
//   I2CLineMon - временные требования Standard-mode и счёт START / STOP;
//   Check      - счётчик ошибок и сравнения.
// Проверки:
//   1. После сброса: SCL = SDA = 1, cmd_ready = 1
//   2. START из свободной шины: SDA = 0, SCL = 0 после него, время 15 мкс, один rsp_valid
//   3. Передача байтов 0xB2, 0x00, 0xFF, 0x80: на проводах ровно эти биты, rsp_bit = отправленный
//   4. Приём байта 0x69 и 0x96 (SDA ведёт тестбенч): rsp_bit совпадает
//   5. Повторный START при SCL = 0 (tSU;STA), затем STOP, шина свободна, снова START
//   6. cmd_ready = 0, пока команда выполняется; бит длится 10 мкс (100 кГц)
//   7. Растяжка SCL 60 мкс после каждого спада: всё выполняется, без timeout, tHIGH соблюдён
//   8. Растяжка 2500 мкс (больше TIMEOUT_US): rsp_timeout, линии отпущены; потом снова работает
//   9. Арбитраж: передача 1 при SDA, удерживаемой в 0 -> rsp_arb; START при SDA = 0 -> rsp_arb
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps
`include "i2c_line_mon.svh"

module i2c_bit_tb;

    localparam int  CLK_FREQ   = 50_000_000;
    localparam real CLK_NS     = 1.0e9 / CLK_FREQ;
    localparam int  TIMEOUT_US = 1_000;

    localparam logic [1:0] BIT_START = 2'd0, BIT_STOP = 2'd1, BIT_WRITE = 2'd2, BIT_READ = 2'd3;

    class Check;
        int errors = 0;

        function void err(string msg);
            errors++;
            if (errors <= 20) $display("[%0t] ERROR: %s", $time, msg);
        endfunction

        function void equal(string name, int got, int exp);
            if (got !== exp) err($sformatf("%s: %0d, expected %0d", name, got, exp));
        endfunction

        function void in_range(string name, real got, real lo, real hi);
            if (got < lo || got > hi) err($sformatf("%s: %0.0f ns, expected %0.0f..%0.0f", name, got, lo, hi));
        endfunction
    endclass

    // -------------------------------------------------------------------------
    // Сигналы и DUT
    // -------------------------------------------------------------------------
    logic       clk = 1'b0, rst_n = 1'b0;
    logic [1:0] cmd = '0;
    logic       cmd_bit = 1'b0, cmd_valid = 1'b0;
    logic       cmd_ready, rsp_valid, rsp_bit, rsp_timeout, rsp_arb;
    logic       scl_low, sda_low;
    logic       tb_scl_low = 1'b0, tb_sda_low = 1'b0;     // линии, которые держит тестбенч
    tri1        scl, sda;

    assign scl = scl_low    ? 1'b0 : 1'bz;
    assign scl = tb_scl_low ? 1'b0 : 1'bz;
    assign sda = sda_low    ? 1'b0 : 1'bz;
    assign sda = tb_sda_low ? 1'b0 : 1'bz;

    always #(CLK_NS / 2) clk = ~clk;

    i2c_bit #(
        .CLK_FREQ   (CLK_FREQ),
        .SCL_FREQ   (100_000),
        .TIMEOUT_US (TIMEOUT_US)
    ) dut (
        .clk (clk), .rst_n (rst_n),
        .cmd (cmd), .cmd_bit (cmd_bit), .cmd_valid (cmd_valid), .cmd_ready (cmd_ready),
        .rsp_valid (rsp_valid), .rsp_bit (rsp_bit), .rsp_timeout (rsp_timeout), .rsp_arb (rsp_arb),
        .scl_in (scl), .sda_in (sda), .scl_low (scl_low), .sda_low (sda_low)
    );

    Check      chk = new();
    I2CLineMon mon = new();

    // события при сбросе (линии выходят из X) монитор не видит
    always @(posedge scl) if (rst_n) mon.scl_rise($realtime, sda);
    always @(negedge scl) if (rst_n) mon.scl_fall($realtime, sda);
    always @(posedge sda) if (rst_n) mon.sda_rise($realtime, scl);
    always @(negedge sda) if (rst_n) mon.sda_fall($realtime, scl);

    // биты на проводах: SDA в момент каждого фронта SCL
    logic wire_bits [$];
    always @(posedge scl) wire_bits.push_back(sda);

    // число импульсов rsp_valid
    int n_rsp = 0;
    always @(posedge clk) if (rsp_valid) n_rsp++;

    // растяжка SCL: «ведомый» держит SCL после спада
    int stretch_us = 0;
    always @(negedge scl) begin
        if (stretch_us > 0) begin
            tb_scl_low = 1'b1;
            #(stretch_us * 1000.0);
            tb_scl_low = 1'b0;
        end
    end

    // -------------------------------------------------------------------------
    // Команда нижнему уровню и ожидание ответа
    // -------------------------------------------------------------------------
    task automatic do_cmd(input logic [1:0] c, input logic b,
                          output logic r_bit, output logic r_to, output logic r_arb, output real dur);
        real t0;
        int  n0;
        @(posedge clk);
        cmd       <= c;
        cmd_bit   <= b;
        cmd_valid <= 1'b1;
        do @(posedge clk); while (!cmd_ready);
        t0 = $realtime;
        n0 = n_rsp;
        cmd_valid <= 1'b0;
        repeat (3) begin
            @(posedge clk);
            if (cmd_ready && !rsp_valid && n_rsp == n0) chk.err("cmd_ready = 1 while the command is running");
        end
        while (!rsp_valid) @(posedge clk);
        dur   = $realtime - t0;
        r_bit = rsp_bit;
        r_to  = rsp_timeout;
        r_arb = rsp_arb;
        @(posedge clk);
        chk.equal("rsp_valid pulses per command", n_rsp - n0, 1);
    endtask

    task automatic cmd_ok(input logic [1:0] c, input logic b, output logic r_bit, output real dur);
        logic to, arb;
        do_cmd(c, b, r_bit, to, arb, dur);
        if (to || arb) chk.err($sformatf("cmd %0d: timeout = %b arb = %b", c, to, arb));
    endtask

    // байт целиком: 8 команд BIT_WRITE
    task automatic send_byte(input logic [7:0] d);
        logic r;
        real  dur;
        for (int k = 7; k >= 0; k--) begin
            cmd_ok(BIT_WRITE, d[k], r, dur);
            chk.equal($sformatf("write bit %0d readback", k), r, d[k]);
            chk.in_range("write bit time", dur, 9_980.0, 10_300.0 + stretch_us * 1000.0);
        end
    endtask

    // байт целиком: 8 команд BIT_READ, SDA ведёт тестбенч
    task automatic recv_byte(input logic [7:0] d, output logic [7:0] got);
        logic r;
        real  dur;
        got = '0;
        for (int k = 7; k >= 0; k--) begin
            tb_sda_low = !d[k];
            cmd_ok(BIT_READ, 1'b1, r, dur);
            got = {got[6:0], r};
        end
        tb_sda_low = 1'b0;
    endtask

    task automatic expect_wire_bits(input string name, input logic [7:0] d);
        if (wire_bits.size() != 8) begin
            chk.err($sformatf("%s: %0d clock pulses on SCL, expected 8", name, wire_bits.size()));
        end else begin
            for (int k = 0; k < 8; k++)
                if (wire_bits[k] !== d[7 - k])
                    chk.err($sformatf("%s: wire bit %0d = %b, expected %b", name, k, wire_bits[k], d[7 - k]));
        end
    endtask

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        logic r, to, arb;
        real  dur;
        logic [7:0] got;
        logic [7:0] bytes [4];
        $timeformat(-6, 2, " us", 0);
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);

        bytes = '{8'hB2, 8'h00, 8'hFF, 8'h80};
        $display("[%0t] 1. Reset state", $time);
        if (scl !== 1'b1 || sda !== 1'b1 || cmd_ready !== 1'b1)
            chk.err($sformatf("after reset SCL=%b SDA=%b cmd_ready=%b", scl, sda, cmd_ready));

        $display("[%0t] 2. START from idle bus", $time);
        cmd_ok(BIT_START, 1'b0, r, dur);
        chk.in_range("START time", dur, 14_980.0, 15_400.0);
        if (scl !== 1'b0 || sda !== 1'b0) chk.err($sformatf("after START SCL=%b SDA=%b, expected 0 0", scl, sda));
        chk.equal("n_start", mon.n_start, 1);

        $display("[%0t] 3. Write bytes", $time);
        foreach (bytes[i]) begin
            wire_bits.delete();
            send_byte(bytes[i]);
            expect_wire_bits($sformatf("byte %h", bytes[i]), bytes[i]);
        end

        $display("[%0t] 4. Read bytes", $time);
        foreach (bytes[i]) begin
            automatic logic [7:0] pat = (i == 0) ? 8'h69 : (i == 1) ? 8'h96 : bytes[i];
            wire_bits.delete();
            recv_byte(pat, got);
            chk.equal($sformatf("read byte %h", pat), got, pat);
            expect_wire_bits($sformatf("read byte %h", pat), pat);
        end

        $display("[%0t] 5. Repeated START, STOP, START", $time);
        cmd_ok(BIT_START, 1'b0, r, dur);                    // повторный START при SCL = 0
        chk.in_range("repeated START time", dur, 14_980.0, 15_400.0);
        chk.equal("n_start after repeated START", mon.n_start, 2);
        send_byte(8'hA5);
        cmd_ok(BIT_STOP, 1'b0, r, dur);
        chk.in_range("STOP time", dur, 14_980.0, 15_400.0);
        if (scl !== 1'b1 || sda !== 1'b1) chk.err($sformatf("after STOP SCL=%b SDA=%b, expected 1 1", scl, sda));
        if (scl_low !== 1'b0 || sda_low !== 1'b0) chk.err("lines are not released after STOP");
        chk.equal("n_stop", mon.n_stop, 1);
        cmd_ok(BIT_START, 1'b0, r, dur);                    // сразу после STOP (tBUF проверяет монитор)
        cmd_ok(BIT_STOP, 1'b0, r, dur);
        chk.equal("n_start", mon.n_start, 3);
        chk.equal("n_stop", mon.n_stop, 2);

        $display("[%0t] 7. Clock stretching 60 us after every fall", $time);
        mon.check_period = 1'b0;
        stretch_us = 60;
        cmd_ok(BIT_START, 1'b0, r, dur);
        wire_bits.delete();
        send_byte(8'hC3);
        expect_wire_bits("stretched byte", 8'hC3);
        recv_byte(8'h5A, got);
        chk.equal("stretched read", got, 8'h5A);
        cmd_ok(BIT_STOP, 1'b0, r, dur);
        stretch_us = 0;
        #(100_000.0);
        mon.check_period = 1'b1;

        $display("[%0t] 8. Stretching longer than the timeout", $time);
        mon.enable = 1'b0;
        cmd_ok(BIT_START, 1'b0, r, dur);
        stretch_us = 2500;
        do_cmd(BIT_WRITE, 1'b0, r, to, arb, dur);           // первый спад SCL уже был; ждём растяжку на подъёме
        if (!to) begin
            // растяжка начинается со спада SCL в конце команды, поэтому timeout наступит на следующем бите
            do_cmd(BIT_WRITE, 1'b1, r, to, arb, dur);
        end
        chk.equal("rsp_timeout", to, 1);
        chk.equal("rsp_arb", arb, 0);
        chk.in_range("time until timeout", dur, TIMEOUT_US * 1000.0, TIMEOUT_US * 1000.0 + 20_000.0);
        if (scl_low !== 1'b0 || sda_low !== 1'b0) chk.err("lines are not released after timeout");
        stretch_us = 0;
        #(3_000_000.0);                                     // растяжка закончилась
        if (scl !== 1'b1 || sda !== 1'b1) chk.err($sformatf("bus is not idle after timeout SCL=%b SDA=%b", scl, sda));
        cmd_ok(BIT_START, 1'b0, r, dur);                    // модуль работает дальше
        cmd_ok(BIT_STOP, 1'b0, r, dur);
        #(20_000.0);
        mon.enable = 1'b1;

        $display("[%0t] 9. Arbitration", $time);
        mon.enable = 1'b0;
        cmd_ok(BIT_START, 1'b0, r, dur);
        tb_sda_low = 1'b1;                                  // SDA держит кто-то другой
        do_cmd(BIT_WRITE, 1'b1, r, to, arb, dur);
        chk.equal("write 1 against SDA = 0: rsp_arb", arb, 1);
        chk.equal("write 1 against SDA = 0: rsp_timeout", to, 0);
        if (scl_low !== 1'b0 || sda_low !== 1'b0) chk.err("lines are not released after arb");
        tb_sda_low = 1'b0;
        #(20_000.0);
        do_cmd(BIT_START, 1'b0, r, to, arb, dur);           // SCL = 1, SDA = 1 после отпускания
        cmd_ok(BIT_STOP, 1'b0, r, dur);
        tb_sda_low = 1'b1;                                  // SDA залипла в 0 при свободной шине
        #(20_000.0);
        do_cmd(BIT_START, 1'b0, r, to, arb, dur);
        chk.equal("START with SDA stuck low: rsp_arb", arb, 1);
        tb_sda_low = 1'b0;
        #(20_000.0);
        cmd_ok(BIT_START, 1'b0, r, dur);                    // всё снова работает
        cmd_ok(BIT_STOP, 1'b0, r, dur);
        #(20_000.0);
        mon.enable = 1'b1;
        cmd_ok(BIT_START, 1'b0, r, dur);                    // с включённым монитором
        send_byte(8'h3C);
        cmd_ok(BIT_STOP, 1'b0, r, dur);

        $display("------------------------------------------------------------");
        $display("ERRORS: %0d (testbench) + %0d (bus monitor)", chk.errors, mon.errors);
        if (chk.errors == 0 && mon.errors == 0) $display("TEST PASSED");
        else                                    $display("TEST FAILED");
        $display("------------------------------------------------------------");
        $finish;
    end

    initial begin
        #60ms;
        $display("TEST FAILED: global timeout");
        $finish;
    end

endmodule
