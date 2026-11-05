// =============================================================================
// Тестбенч верхнего уровня EE_topB: кнопки, сценарии, реальная микросхема на выводах I2C
//
// К выводам i2c_scl / i2c_sda подключён eeprom_model как «настоящая» AT24C08 (цикл записи
// 200 мкс, память заполнена мусором 0x5A); внутренний имитатор в ПЛИС - второй источник.
// Тайминги уменьшены: антидребезг 20 тактов, вспышка светодиода 200 тактов, кольцо 16 записей.
// Проверки:
//   1. KEY2 листает сценарии 0..9 по кругу; KEY1 запускает выбранный; во время выполнения
//      KEY1 и KEY4 игнорируются
//   2. CLEAR стирает первые 8 слов микросхемы (были 0x5A5A)
//   3. Три PUSH пишут в микросхему 1960, 1980, 19A0 по адресам 0, 2, 4 (в её памяти)
//   4. Сброс платы: указатели кольца, курсор, значение записей в начале, а содержимое микросхемы
//      сохранилось; три RD_WORD читают те же слова по адресам 0, 2, 4 (курсор +2), info_data
//      и режим индикатора «адрес/данные»
//   5. KEY4 переключает источник на имитатор в ПЛИС: чтение из него даёт 0xFFFF (новая память),
//      на внешней шине в это время нет событий
//   6. Микросхемы нет (present = 0): сценарий кончается ошибкой, led[0] (ERR) горит, KEY4 гасит
//   7. KEY3 меняет режим индикатора; режим сценария показывается после KEY2
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module EE_topB_tb;

    localparam int  CLK_FREQ = 50_000_000;
    localparam real CLK_NS   = 1.0e9 / CLK_FREQ;
    localparam int  DEB      = 20;

    class Scoreboard;
        int errors = 0;

        function void err(string msg);
            errors++;
            if (errors <= 25) $display("[%0t] ERROR: %s", $time, msg);
        endfunction
    endclass

    logic        clk = 1'b0, rst_n = 1'b0;
    logic [3:0]  key = 4'b1111;
    tri1         scl, sda;
    logic [7:0]  seg;
    logic [3:0]  dig, led;
    logic        chip_present = 1'b1, chip_busy, chip_sda_low;
    logic [15:0] chip_cycles;

    always #(CLK_NS / 2) clk = ~clk;

    EE_topB #(
        .CLK_FREQ       (CLK_FREQ),
        .RECORDS        (16),
        .MODEL_WRITE_US (200),
        .DEB_CLKS       (DEB),
        .AUTO_CLKS      (4000),
        .DIGIT_HZ       (CLK_FREQ / 20),
        .BLANK_CLKS     (2),
        .FLASH_CLKS     (200)
    ) dut (
        .clk (clk), .rst_n (rst_n), .key (key), .i2c_scl (scl), .i2c_sda (sda),
        .seg (seg), .dig (dig), .led (led)
    );

    // «настоящая» микросхема на выводах I2C
    eeprom_model #(
        .CLK_FREQ (CLK_FREQ),
        .ADDR     (7'h50),
        .WRITE_US (200)
    ) chip (
        .clk (clk), .rst_n (rst_n), .present (chip_present), .busy (chip_busy), .wr_cycles (chip_cycles),
        .scl_in (scl), .sda_in (sda), .sda_low (chip_sda_low)
    );
    assign sda = chip_sda_low ? 1'b0 : 1'bz;

    Scoreboard sb = new();

    // события на внешней шине
    int ext_starts = 0;
    always @(negedge sda) if (scl === 1'b1) ext_starts++;

    // импульсы START сценариев
    int n_fin = 0;
    always @(posedge clk) if (dut.u_seq.finished) n_fin++;

    task automatic press(input int k);
        key[k] = 1'b0;
        repeat (3 * DEB) @(posedge clk);
        key[k] = 1'b1;
        repeat (3 * DEB) @(posedge clk);
    endtask

    task automatic wait_finished();
        int guard;
        guard = 0;
        while (dut.u_seq.busy && guard < 20_000_000) begin
            @(posedge clk);
            guard++;
        end
        if (guard >= 20_000_000) sb.err("scenario does not finish");
        repeat (20) @(posedge clk);
    endtask

    task automatic run_current();
        int n0;
        n0 = n_fin;
        press(0);
        wait_finished();
        if (n_fin != n0 + 1) sb.err($sformatf("%0d finished pulses for one KEY1", n_fin - n0));
    endtask

    task automatic select_script(input int target);
        int guard;
        guard = 0;
        while (int'(dut.script) != target && guard < 12) begin
            press(1);
            guard++;
        end
        if (int'(dut.script) != target) sb.err($sformatf("script %0d cannot be selected", dut.script));
    endtask

    task automatic reset_board();
        rst_n = 1'b0;
        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);
    endtask

    task automatic check_word(input string name, input int addr, input logic [15:0] w);
        logic [7:0] hi, lo;
        hi = chip.mem[addr];
        lo = chip.mem[addr + 1];
        if ({hi, lo} !== w) sb.err($sformatf("%s: chip word at %0d = %h, expected %h", name, addr, {hi, lo}, w));
    endtask

    initial begin
        int  s0, c0, k;
        for (int i = 0; i < 1024; i++) chip.mem[i] = 8'h5A;      // мусор в «микросхеме»
        $timeformat(-6, 2, " us", 0);
        #(CLK_NS * 5);
        rst_n = 1'b1;
        repeat (20) @(posedge clk);

        // ---------------------------------------------------------------- 1
        if (dut.script !== 4'd0 || dut.use_model !== 1'b0) sb.err("state after reset: script 0, real chip");
        for (k = 1; k <= 10; k++) begin
            press(1);
            if (int'(dut.script) != k % 10) sb.err($sformatf("KEY2 press %0d: script %0d", k, dut.script));
        end
        if (int'(dut.u_ind.mode) != 9) sb.err("KEY2 did not show the script screen");
        press(2);
        if (int'(dut.u_ind.mode) != 0) sb.err($sformatf("KEY3 from the script screen: mode %0d, expected 0 (auto)", dut.u_ind.mode));
        press(2);
        if (int'(dut.u_ind.mode) != 1) sb.err($sformatf("KEY3: mode %0d, expected 1", dut.u_ind.mode));

        // ---------------------------------------------------------------- 2
        select_script(4);                                       // CLEAR
        run_current();
        for (int w = 0; w < 8; w++) check_word("CLEAR", 2 * w, 16'hFFFF);
        check_word("CLEAR leaves the rest", 16, 16'h5A5A);
        if (dut.u_ind.mode != 0) sb.err("KEY1 did not return the auto mode");

        // KEY1 и KEY4 во время выполнения игнорируются
        select_script(3);                                       // FILL: 16 + 8 записей
        c0 = chip_cycles;
        press(0);
        repeat (2000) @(posedge clk);
        if (!dut.u_seq.busy) sb.err("FILL did not start");
        press(0);                                               // KEY1 ещё раз
        press(3);                                               // KEY4: источник не должен смениться
        if (dut.use_model) sb.err("KEY4 changed the source during a scenario");
        wait_finished();
        if (chip_cycles - c0 != 24) sb.err($sformatf("FILL made %0d write cycles, expected 24 (second KEY1 ignored)", chip_cycles - c0));
        if (dut.ring_ovf !== 1'b1) sb.err("FILL: no ovf");

        // ---------------------------------------------------------------- 3
        select_script(0);                                       // START
        run_current();
        select_script(1);                                       // PUSH
        for (int i = 0; i < 3; i++) run_current();
        check_word("PUSH 0", 0, 16'h1960);
        check_word("PUSH 1", 2, 16'h1980);
        check_word("PUSH 2", 4, 16'h19A0);
        if (dut.u_seq.info_data !== 16'h19A0 || dut.u_seq.info_addr !== 10'd4) sb.err("info after PUSH");
        if (dut.ring_count !== 5'd3) sb.err($sformatf("ring count %0d, expected 3", dut.ring_count));

        // ---------------------------------------------------------------- 4
        reset_board();
        if (dut.ring_count !== 5'd0 || dut.ring_wr_ptr !== 4'd0 || dut.ring_rd_ptr !== 4'd0 || dut.u_seq.cursor !== 10'd0)
            sb.err("pointers are not reset by the board reset");
        check_word("after reset 0", 0, 16'h1960);
        check_word("after reset 1", 2, 16'h1980);
        check_word("after reset 2", 4, 16'h19A0);
        select_script(8);                                       // RD_WORD
        for (int i = 0; i < 3; i++) begin
            run_current();
            if (dut.u_seq.info_data !== (16'h1960 + 16'(i) * 16'h0020))
                sb.err($sformatf("RD_WORD %0d after reset: %h", i, dut.u_seq.info_data));
            if (dut.u_seq.info_addr !== 10'(2 * i)) sb.err($sformatf("RD_WORD %0d: address %h", i, dut.u_seq.info_addr));
        end
        if (dut.u_seq.cursor !== 10'd6) sb.err($sformatf("cursor %0d after 3 RD_WORD, expected 6", dut.u_seq.cursor));
        select_script(2);                                       // POP: после сброса записей в кольце нет
        run_current();
        if (!dut.u_seq.info_empty) sb.err("POP after the reset returned an old record");

        // ---------------------------------------------------------------- 5
        select_script(8);
        press(3);                                               // источник: имитатор
        if (!dut.use_model) sb.err("KEY4 did not select the simulator");
        s0 = ext_starts;
        run_current();
        if (dut.u_seq.info_data !== 16'hFFFF) sb.err($sformatf("RD_WORD from a new simulator = %h", dut.u_seq.info_data));
        if (ext_starts != s0) sb.err("the simulator generated events on the pins");
        press(3);                                               // обратно к микросхеме
        if (dut.use_model) sb.err("KEY4 did not select the real chip");

        // ---------------------------------------------------------------- 6
        chip_present = 1'b0;
        select_script(8);
        run_current();
        if (!dut.u_seq.error) sb.err("no error without the chip");
        if (led[0] !== 1'b0) sb.err("led[0] (ERR) is off after the error");
        chip_present = 1'b1;
        press(3);                                               // KEY4 гасит ERR (и меняет источник)
        press(3);
        if (led[0] !== 1'b1) sb.err("KEY4 did not clear ERR");
        run_current();
        if (dut.u_seq.error) sb.err("error after the chip returned");

        repeat (20) @(posedge clk);
        if (sb.errors == 0) $display("TEST PASSED");
        else                $display("TEST FAILED: %0d errors", sb.errors);
        $finish;
    end

endmodule
