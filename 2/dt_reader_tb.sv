// =============================================================================
// Тестбенч для dt_reader: чтение температуры через i2c_master, 50 МГц, SCL 100 кГц
//
// Три независимых окружения dt_rd_env (читатель + i2c_master + своя шина):
//   0 (A) - датчик dt_model (имитатор LM75A), WRITE_CONFIG = 1;
//   1 (B) - независимая модель i2c_slave_model (память + указатель), WRITE_CONFIG = 0;
//   2 (C) - та же модель, WRITE_CONFIG = 1 (смотрим, что именно записано в датчик).
// Класс TempHist (история температуры внутри dt_model) проверяет, что прочитанное
// значение было текущим во время чтения. Тестбенч записывает команды, поданные
// читателем на мастер: адрес, чтение / запись, число байт, STOP.
// Проверки:
//   A1. Конфигурация shutdown (принудительно записана в модель) после первого чтения
//       сброшена в 0x00; команды: запись 2 байт, запись указателя без STOP, чтение 2 байт
//   A2. Следующие чтения - без записи конфигурации (только 2 команды)
//   A3. Постоянная температура -1.0 °C: 0xFF00 (знак сохраняется); 60 чтений пилы в
//       случайные моменты: каждое значение было текущим в течение чтения
//   A4. start_read во время чтения игнорируется: один temp_valid
//   A5. Датчик отключён (present = 0): read_fail без temp_valid, temp_data не меняется;
//       после включения чтение работает
//   B1. Чтение 40 случайных значений из памяти модели: {старший, младший & 0xE0};
//       записан только указатель 0x00; temp_valid и read_fail - импульсы по 1 такту
//   B2. NACK на байт данных (wr_limit = 0): read_fail; растяжка SCL дольше предела:
//       read_fail (timeout); SDA залипла: read_fail (arb); после снятия неисправности
//       чтение снова работает; сбой (тайм-аут) на втором байте при уже принятом первом -
//       тоже read_fail, temp_data не меняется
//   C1. Первое чтение: в датчик записано 0x01 0x00 (конфигурация), затем указатель 0x00;
//       второе чтение конфигурацию уже не пишет
//   C2. Датчик не ответил на запись конфигурации: read_fail; при следующем чтении
//       конфигурация записывается снова; NACK на запись конфигурации
//       (wr_limit = 1) тоже read_fail
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// Окружение: читатель + мастер на своей шине (только для тестбенча)
// -----------------------------------------------------------------------------
module dt_rd_env #(
    parameter bit WRITE_CONFIG = 1'b1
) (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start_read,
    output logic        busy,
    output logic        temp_valid,
    output logic [15:0] temp_data,
    output logic        read_fail,
    output logic        cmd_fire,
    output logic [16:0] cmd_info,           // {адрес[6:0], чтение, число байт[7:0], STOP}
    inout  tri1         scl,
    inout  tri1         sda
);

    localparam int LEN_W = 8;

    logic             cmd_valid, cmd_ready, cmd_read, cmd_stop;
    logic [6:0]       cmd_addr;
    logic [LEN_W-1:0] cmd_len;
    logic [7:0]       wr_data, rd_data;
    logic             wr_valid, wr_ready, rd_valid, rd_ready, rd_last;
    logic             done, res_nack_addr, res_nack_data, res_timeout, res_arb;
    logic [LEN_W-1:0] res_len;

    dt_reader #(
        .LEN_W        (LEN_W),
        .DT_ADDR      (7'h48),
        .WRITE_CONFIG (WRITE_CONFIG)
    ) u_reader (
        .clk (clk), .rst_n (rst_n),
        .start_read (start_read), .busy (busy), .temp_valid (temp_valid),
        .temp_data (temp_data), .read_fail (read_fail),
        .cmd_valid (cmd_valid), .cmd_ready (cmd_ready), .cmd_addr (cmd_addr), .cmd_read (cmd_read),
        .cmd_len (cmd_len), .cmd_stop (cmd_stop),
        .wr_data (wr_data), .wr_valid (wr_valid), .wr_ready (wr_ready),
        .rd_data (rd_data), .rd_valid (rd_valid), .rd_ready (rd_ready), .rd_last (rd_last),
        .done (done), .res_nack_addr (res_nack_addr), .res_nack_data (res_nack_data),
        .res_timeout (res_timeout), .res_arb (res_arb)
    );

    i2c_master #(
        .CLK_FREQ   (50_000_000),
        .SCL_FREQ   (100_000),
        .TIMEOUT_US (1_000),
        .LEN_W      (LEN_W)
    ) u_master (
        .clk (clk), .rst_n (rst_n),
        .cmd_valid (cmd_valid), .cmd_ready (cmd_ready), .cmd_addr (cmd_addr), .cmd_read (cmd_read),
        .cmd_len (cmd_len), .cmd_stop (cmd_stop),
        .wr_data (wr_data), .wr_valid (wr_valid), .wr_ready (wr_ready),
        .rd_data (rd_data), .rd_valid (rd_valid), .rd_ready (rd_ready), .rd_last (rd_last),
        .done (done), .res_len (res_len), .res_nack_addr (res_nack_addr), .res_nack_data (res_nack_data),
        .res_timeout (res_timeout), .res_arb (res_arb),
        .ev_start (), .ev_stop (), .ev_ack (), .ev_nack (),
        .i2c_scl (scl), .i2c_sda (sda)
    );

    assign cmd_fire = cmd_valid && cmd_ready;
    assign cmd_info = {cmd_addr, cmd_read, cmd_len, cmd_stop};

endmodule

// -----------------------------------------------------------------------------
module dt_reader_tb;

    localparam int  CLK_FREQ = 50_000_000;
    localparam real CLK_NS   = 1.0e9 / CLK_FREQ;
    localparam int  N_ENV    = 3;

    typedef logic [16:0] cmd_t;
    typedef logic [7:0]  bytes_t[$];

    function automatic cmd_t mkc(logic [6:0] a, bit rd, int len, bit stop);
        return {a, rd, 8'(len), stop};
    endfunction

    class TempHist;
        realtime     t [$];
        logic [15:0] v [$];

        function void add(realtime tm, logic [15:0] val);
            t.push_back(tm);
            v.push_back(val);
        endfunction

        function bit seen(realtime t0, realtime t1, logic [15:0] val);
            for (int i = 0; i < t.size(); i++) begin
                realtime t_end;
                t_end = (i + 1 < t.size()) ? t[i + 1] : 1.0e18;
                if (t[i] <= t1 && t_end >= t0 && v[i] === val) return 1'b1;
            end
            return 1'b0;
        endfunction
    endclass

    class Scoreboard;
        int errors = 0;

        function void err(string msg);
            errors++;
            if (errors <= 20) $display("[%0t] ERROR: %s", $time, msg);
        endfunction
    endclass

    // -------------------------------------------------------------------------
    // Сигналы
    // -------------------------------------------------------------------------
    logic              clk = 1'b0, rst_n = 1'b0;
    logic [N_ENV-1:0]  start_read = '0, busy, temp_valid, read_fail, cmd_fire;
    logic [15:0]       temp_data [N_ENV];
    cmd_t              cmd_info  [N_ENV];
    tri1               scl [N_ENV];
    tri1               sda [N_ENV];

    logic               present = 1'b1, gen_restart = 1'b0;
    logic        [1:0]  gen_mode = '0;
    logic signed [10:0] gen_lo = -11'sd8, gen_hi = 11'sd1000;
    logic        [10:0] gen_step = 11'd8;
    logic        [7:0]  gen_noise_mask = '0;
    logic               model_sda_low;

    always #(CLK_NS / 2) clk = ~clk;

    dt_rd_env #(.WRITE_CONFIG (1'b1)) env_a (
        .clk (clk), .rst_n (rst_n), .start_read (start_read[0]), .busy (busy[0]),
        .temp_valid (temp_valid[0]), .temp_data (temp_data[0]), .read_fail (read_fail[0]),
        .cmd_fire (cmd_fire[0]), .cmd_info (cmd_info[0]), .scl (scl[0]), .sda (sda[0])
    );
    dt_rd_env #(.WRITE_CONFIG (1'b0)) env_b (
        .clk (clk), .rst_n (rst_n), .start_read (start_read[1]), .busy (busy[1]),
        .temp_valid (temp_valid[1]), .temp_data (temp_data[1]), .read_fail (read_fail[1]),
        .cmd_fire (cmd_fire[1]), .cmd_info (cmd_info[1]), .scl (scl[1]), .sda (sda[1])
    );
    dt_rd_env #(.WRITE_CONFIG (1'b1)) env_c (
        .clk (clk), .rst_n (rst_n), .start_read (start_read[2]), .busy (busy[2]),
        .temp_valid (temp_valid[2]), .temp_data (temp_data[2]), .read_fail (read_fail[2]),
        .cmd_fire (cmd_fire[2]), .cmd_info (cmd_info[2]), .scl (scl[2]), .sda (sda[2])
    );

    dt_model #(
        .CLK_FREQ  (CLK_FREQ),
        .ADDR      (7'h48),
        .UPDATE_MS (1)
    ) model_a (
        .clk (clk), .rst_n (rst_n),
        .present (present), .gen_restart (gen_restart), .gen_mode (gen_mode),
        .gen_lo (gen_lo), .gen_hi (gen_hi), .gen_step (gen_step), .gen_noise_mask (gen_noise_mask),
        .scl_in (scl[0]), .sda_in (sda[0]), .sda_low (model_sda_low)
    );
    assign sda[0] = model_sda_low ? 1'b0 : 1'bz;

    i2c_slave_model #(.ADDR (7'h48)) slave_b (.scl (scl[1]), .sda (sda[1]));
    i2c_slave_model #(.ADDR (7'h48)) slave_c (.scl (scl[2]), .sda (sda[2]));

    Scoreboard sb   = new();
    TempHist   hist = new();

    always @(model_a.temp_word) hist.add($realtime, model_a.temp_word);

    // журнал команд, подаваемых читателем на мастер
    cmd_t clog [N_ENV][$];

    always @(posedge clk) begin
        for (int i = 0; i < N_ENV; i++)
            if (cmd_fire[i]) clog[i].push_back(cmd_info[i]);
    end

    // подсчёт импульсов и проверка их ширины
    int n_valid [N_ENV];
    int n_fail  [N_ENV];
    logic [N_ENV-1:0] valid_q = '0, fail_q = '0;

    always @(posedge clk) begin
        for (int i = 0; i < N_ENV; i++) begin
            if (temp_valid[i]) n_valid[i]++;
            if (read_fail[i])  n_fail[i]++;
            if (temp_valid[i] && valid_q[i]) sb.err($sformatf("env %0d: temp_valid is wider than 1 clock", i));
            if (read_fail[i]  && fail_q[i])  sb.err($sformatf("env %0d: read_fail is wider than 1 clock", i));
            if (temp_valid[i] && read_fail[i]) sb.err($sformatf("env %0d: temp_valid and read_fail together", i));
        end
        valid_q <= temp_valid;
        fail_q  <= read_fail;
    end

    // -------------------------------------------------------------------------
    // Одно чтение: импульс start_read, ожидание конца
    // -------------------------------------------------------------------------
    bit          r_ok, r_failed;
    logic [15:0] r_data;
    realtime     r_t0, r_t1;

    task automatic do_read(input int idx);
        int v0, f0, guard;
        v0 = n_valid[idx];
        f0 = n_fail[idx];
        clog[idx].delete();
        r_t0 = $realtime;
        @(posedge clk);
        start_read[idx] <= 1'b1;
        @(posedge clk);
        start_read[idx] <= 1'b0;
        guard = 0;
        do begin
            @(posedge clk);
            guard++;
        end while ((busy[idx] || guard < 4) && guard < 400_000);
        if (guard >= 400_000) sb.err($sformatf("env %0d: read does not finish", idx));
        repeat (3) @(posedge clk);
        r_t1     = $realtime;
        r_ok     = (n_valid[idx] - v0 == 1) && (n_fail[idx] == f0);
        r_failed = (n_fail[idx]  - f0 == 1) && (n_valid[idx] == v0);
        r_data   = temp_data[idx];
        if (!r_ok && !r_failed)
            sb.err($sformatf("env %0d: %0d temp_valid and %0d read_fail pulses for one read", idx,
                             n_valid[idx] - v0, n_fail[idx] - f0));
    endtask

    task automatic wait_ms(input int ms);
        #(ms * 1_000_000.0);
    endtask

    task automatic check_cmds(input string name, input int idx, input cmd_t exp[$]);
        if (clog[idx].size() != exp.size())
            sb.err($sformatf("%s: %0d commands, expected %0d: %p", name, clog[idx].size(), exp.size(), clog[idx]));
        else
            for (int i = 0; i < exp.size(); i++)
                if (clog[idx][i] !== exp[i])
                    sb.err($sformatf("%s: command %0d = %h, expected %h", name, i, clog[idx][i], exp[i]));
    endtask

    task automatic set_gen(input int mode, input int lo, input int hi, input int step);
        @(negedge clk);
        gen_mode = mode;  gen_lo = lo;  gen_hi = hi;  gen_step = step;
        @(negedge clk);
        gen_restart = 1'b1;
        @(negedge clk);
        gen_restart = 1'b0;
    endtask

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        cmd_t        exp[$];
        logic [15:0] x, prev;
        logic [7:0]  m0, m1;
        int          n_bad, v0;
        for (int i = 0; i < N_ENV; i++) begin n_valid[i] = 0; n_fail[i] = 0; end
        $timeformat(-6, 2, " us", 0);
        hist.add(0, '0);
        for (int i = 0; i < 256; i++) begin slave_b.mem[i] = '0; slave_c.mem[i] = '0; end
        #(CLK_NS * 5);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        // ---------------------------------------------------------------- A1
        model_a.conf = 8'h01;                                  // датчик «спит»
        set_gen(0, -8, 1000, 8);                               // -1.0 °C
        wait_ms(2);
        do_read(0);
        if (!r_ok) sb.err("A1: first read failed");
        if (r_data !== 16'hFF00) sb.err($sformatf("A1: -1.0 degC read as %h, expected FF00", r_data));
        if (model_a.conf !== 8'h00) sb.err($sformatf("A1: Config = %h after first read, expected 00", model_a.conf));
        exp.delete();
        exp.push_back(mkc(7'h48, 0, 2, 1));
        exp.push_back(mkc(7'h48, 0, 1, 0));
        exp.push_back(mkc(7'h48, 1, 2, 1));
        check_cmds("A1 commands", 0, exp);

        // ---------------------------------------------------------------- A2
        do_read(0);
        if (!r_ok || r_data !== 16'hFF00) sb.err($sformatf("A2: second read %h", r_data));
        exp.delete();
        exp.push_back(mkc(7'h48, 0, 1, 0));
        exp.push_back(mkc(7'h48, 1, 2, 1));
        check_cmds("A2 commands", 0, exp);

        // ---------------------------------------------------------------- A3
        set_gen(0, 203, 1000, 8);                              // 25.375 °C
        wait_ms(2);
        do_read(0);
        if (r_data !== 16'h1960) sb.err($sformatf("A3: 25.375 degC read as %h, expected 1960", r_data));
        set_gen(2, -1000, 1000, 311);                          // пила с большим шагом
        n_bad = 0;
        for (int i = 0; i < 60; i++) begin
            #($urandom_range(0, 1_500_000));
            do_read(0);
            if (!r_ok) sb.err($sformatf("A3: read %0d failed", i));
            else if (!hist.seen(r_t0, r_t1, r_data)) begin
                n_bad++;
                sb.err($sformatf("A3: read %0d value %h was not current during the read", i, r_data));
            end
        end

        // ---------------------------------------------------------------- A4
        set_gen(0, 203, 1000, 8);
        wait_ms(2);
        v0 = n_valid[0];
        @(posedge clk);
        start_read[0] <= 1'b1;
        @(posedge clk);
        repeat (20) @(posedge clk);                            // удерживаем start_read во время чтения
        start_read[0] <= 1'b0;
        wait_ms(2);
        if (n_valid[0] - v0 != 1)
            sb.err($sformatf("A4: %0d temp_valid pulses, expected 1 (start_read during busy)", n_valid[0] - v0));

        // ---------------------------------------------------------------- A5
        x = temp_data[0];
        present = 1'b0;
        do_read(0);
        if (!r_failed) sb.err("A5: no read_fail with sensor off");
        if (temp_data[0] !== x) sb.err("A5: temp_data changed after failed read");
        present = 1'b1;
        do_read(0);
        if (!r_ok || r_data !== 16'h1960) sb.err($sformatf("A5: read after switching on %h", r_data));

        // ---------------------------------------------------------------- B1
        for (int i = 0; i < 40; i++) begin
            m0 = $urandom;  m1 = $urandom;
            if (i == 0) begin m0 = 8'hE7; m1 = 8'h00; end      // -25 °C
            slave_b.mem[0] = m0;  slave_b.mem[1] = m1;
            slave_b.rx_log.delete();
            do_read(1);
            if (!r_ok) sb.err($sformatf("B1: read %0d failed", i));
            if (r_data !== {m0, m1 & 8'hE0})
                sb.err($sformatf("B1: read %0d = %h, expected %h", i, r_data, {m0, m1 & 8'hE0}));
            if (slave_b.rx_log.size() != 1 || slave_b.rx_log[0] !== 8'h00)
                sb.err($sformatf("B1: bytes written to sensor: %p, expected {00}", slave_b.rx_log));
        end

        // ---------------------------------------------------------------- B2
        slave_b.mem[0] = 8'h19;  slave_b.mem[1] = 8'h60;
        x = temp_data[1];
        slave_b.wr_limit = 0;                                  // NACK на байт указателя
        do_read(1);
        if (!r_failed) sb.err("B2: no read_fail on NACK of the pointer byte");
        if (temp_data[1] !== x) sb.err("B2: temp_data changed after failed read");
        slave_b.wr_limit = 1_000_000;
        do_read(1);
        if (!r_ok || r_data !== 16'h1960) sb.err($sformatf("B2: read after NACK = %h", r_data));

        slave_b.stretch_ns = 1_500_000;                        // дольше TIMEOUT_US = 1000
        do_read(1);
        if (!r_failed) sb.err("B2: no read_fail on SCL stretch timeout");
        slave_b.stretch_ns = 0;
        wait_ms(3);
        do_read(1);
        if (!r_ok || r_data !== 16'h1960) sb.err($sformatf("B2: read after timeout = %h", r_data));

        slave_b.hold_sda = 1'b1;                               // SDA залипла
        do_read(1);
        if (!r_failed) sb.err("B2: no read_fail with SDA stuck low");
        slave_b.hold_sda = 1'b0;
        wait_ms(1);
        do_read(1);
        if (!r_ok || r_data !== 16'h1960) sb.err($sformatf("B2: read after stuck SDA = %h", r_data));

        // сбой посреди чтения: первый байт получен, на втором SCL удерживается дольше предела
        slave_b.tx_log.delete();
        x = temp_data[1];
        fork
            begin
                wait (slave_b.tx_log.size() >= 2);
                slave_b.stretch_ns = 1_500_000;
            end
        join_none
        do_read(1);
        disable fork;
        slave_b.stretch_ns = 0;
        if (!r_failed) sb.err("B2: no read_fail on a timeout in the second byte");
        if (temp_data[1] !== x) sb.err("B2: temp_data changed after a failure in the second byte");
        wait_ms(3);
        do_read(1);
        if (!r_ok || r_data !== 16'h1960) sb.err($sformatf("B2: read after a failure in the second byte = %h", r_data));

        // ---------------------------------------------------------------- C1
        slave_c.mem[0] = 8'hE7;
        slave_c.rx_log.delete();
        do_read(2);
        if (!r_ok) sb.err("C1: first read failed");
        if (slave_c.rx_log.size() != 3 || slave_c.rx_log[0] !== 8'h01 || slave_c.rx_log[1] !== 8'h00
            || slave_c.rx_log[2] !== 8'h00)
            sb.err($sformatf("C1: bytes written to sensor: %p, expected {01 00 00}", slave_c.rx_log));
        if (r_data !== 16'hE700) sb.err($sformatf("C1: read %h, expected E700 (Config byte 00 went to mem[1])", r_data));
        slave_c.rx_log.delete();
        slave_c.mem[1] = 8'h20;
        do_read(2);
        if (!r_ok || r_data !== 16'hE720) sb.err($sformatf("C1: second read %h, expected E720", r_data));
        if (slave_c.rx_log.size() != 1 || slave_c.rx_log[0] !== 8'h00)
            sb.err($sformatf("C1: second read wrote %p, expected {00}", slave_c.rx_log));

        // ---------------------------------------------------------------- C2
        rst_n = 1'b0;                                          // сброс: конфигурация снова не записана
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        repeat (3) @(posedge clk);
        slave_c.absent = 1'b1;
        do_read(2);
        if (!r_failed) sb.err("C2: no read_fail with sensor absent");
        exp.delete();
        exp.push_back(mkc(7'h48, 0, 2, 1));                   // дальше NACK на адрес, остальных команд нет
        check_cmds("C2 commands (absent)", 2, exp);
        slave_c.absent = 1'b0;
        slave_c.rx_log.delete();
        do_read(2);
        if (!r_ok) sb.err("C2: read after sensor appeared failed");
        if (slave_c.rx_log.size() != 3 || slave_c.rx_log[0] !== 8'h01)
            sb.err($sformatf("C2: configuration not rewritten after a failure: %p", slave_c.rx_log));

        rst_n = 1'b0;
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        repeat (3) @(posedge clk);
        slave_c.wr_limit = 1;                                  // указатель принят, значение конфигурации - NACK
        do_read(2);
        if (!r_failed) sb.err("C2: no read_fail on NACK of the config value");
        slave_c.wr_limit = 1_000_000;
        slave_c.rx_log.delete();
        do_read(2);
        if (!r_ok || slave_c.rx_log.size() != 3 || slave_c.rx_log[0] !== 8'h01)
            sb.err($sformatf("C2: recovery after config NACK: ok=%b log=%p", r_ok, slave_c.rx_log));

        repeat (10) @(posedge clk);
        if (sb.errors == 0) $display("TEST PASSED");
        else                $display("TEST FAILED: %0d errors", sb.errors);
        $finish;
    end

endmodule
