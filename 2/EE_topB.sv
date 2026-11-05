// =============================================================================
// Проверка EEPROM на плате B: запись, чтение, кольцевой буфер, индикация адреса и данных
//
// Цепочка: кнопки -> eeprom_seq (сценарии) -> eeprom_store (кольцо + eeprom_ctrl)
// -> eeprom_bus (i2c_master + выбор шины: выводы I2C с AT24C08 либо внутренний
// имитатор eeprom_model) -> eeprom_ind (адрес и данные на индикаторе, светодиоды
// WR / RD / BUSY / ERR). Шина I2C: SCL PIN_99, SDA PIN_98.
//
// Кнопки (после подавления дребезга, по нажатию):
//   KEY1 - выполнить выбранный сценарий (во время выполнения игнорируется);
//   KEY2 - следующий сценарий 0..9 (на индикаторе S.   N):
//          0 START, 1 PUSH, 2 POP, 3 FILL, 4 CLEAR, 5 WR_BYTE, 6 WR_WORD,
//          7 RD_BYTE, 8 RD_WORD, 9 RD_SEQ (подробности - в eeprom_seq.sv);
//   KEY3 - следующий режим индикатора (адрес/данные, курсор, указатели, ...), eeprom_ind.sv;
//   KEY4 - источник: реальная микросхема <-> имитатор в ПЛИС (обнуляет счётчик ошибок).
// Кнопка RESET сбрасывает ПЛИС (указатели кольца, курсор, значение gen_val), но не
// трогает содержимое EEPROM: так проверяется сохранность данных.
// =============================================================================
module EE_topB #(
    parameter int         CLK_FREQ       = 50_000_000,
    parameter int         SCL_FREQ       = 100_000,
    parameter int         TIMEOUT_US     = 1_000,
    parameter logic [6:0] EE_ADDR        = 7'h50,        // AT24C08, блок 0 (A2 = 0)
    parameter int         RECORDS        = 512,          // записей в кольце (PROTOCOL.md, раздел 8.1)
    parameter int         MODEL_WRITE_US = 5_000,        // цикл записи имитатора, мкс
    parameter int         POLL_MAX       = 100,          // предел опросов ACK после записи
    parameter int         DEB_CLKS       = CLK_FREQ / 1000 * 10,   // антидребезг 10 мс
    parameter int         AUTO_CLKS      = CLK_FREQ,               // экран в режиме АВТО, 1 с
    parameter int         DIGIT_HZ       = 1_000,
    parameter int         BLANK_CLKS     = 100,
    parameter int         FLASH_CLKS     = CLK_FREQ / 20,          // вспышка светодиода 50 мс
    parameter bit         SEG_ACTIVE     = 1'b0,
    parameter bit         DIG_ACTIVE     = 1'b0,
    parameter bit         LED_ACTIVE     = 1'b0
) (
    input  logic       clk,
    input  logic       rst_n,       // кнопка сброса, нажата = 0
    input  logic [3:0] key,         // KEY1..KEY4, нажата = 0

    inout  wire        i2c_scl,
    inout  wire        i2c_sda,

    output logic [7:0] seg,
    output logic [3:0] dig,
    output logic [3:0] led
);

    localparam int LEN_W    = 8;
    localparam int ADDR_W   = 10;                       // AT24C08: 1024 байта
    localparam int PTR_W    = $clog2(RECORDS);
    localparam int N_SCRIPT = 10;

    // -------------------------------------------------------------------------
    // Синхронизация сброса: асинхронная установка, синхронное снятие
    // -------------------------------------------------------------------------
    logic [1:0] rst_sync;
    logic       rst_n_s;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) rst_sync <= 2'b00;
        else        rst_sync <= {rst_sync[0], 1'b1};
    end

    assign rst_n_s = rst_sync[1];

    // -------------------------------------------------------------------------
    // Кнопки
    // -------------------------------------------------------------------------
    logic [3:0] key_press;

    genvar i;   // Quartus Standard не понимает genvar внутри for

    generate
    for (i = 0; i < 4; i++) begin : g_key
        debounce #(
            .CLK_FREQ    (CLK_FREQ),
            .STABLE_CLKS (DEB_CLKS),
            .ACTIVE      (1'b0)
        ) u_deb (
            .clk       (clk),
            .rst_n     (rst_n_s),
            .btn       (key[i]),
            .pressed   (),
            .press_p   (key_press[i]),
            .release_p ()
        );
    end
    endgenerate

    // -------------------------------------------------------------------------
    // Выбор сценария и источника
    // -------------------------------------------------------------------------
    logic [3:0] script;
    logic       use_model;
    logic       seq_busy;
    logic       run_p, script_changed, source_changed;

    assign run_p          = key_press[0] && !seq_busy;
    assign source_changed = key_press[3] && !seq_busy;

    always_ff @(posedge clk or negedge rst_n_s) begin
        if (!rst_n_s) begin
            script          <= '0;
            use_model       <= 1'b0;
            script_changed  <= 1'b0;
        end else begin
            script_changed <= key_press[1];
            if (key_press[1]) script <= (script == 4'(N_SCRIPT - 1)) ? 4'd0 : script + 1'b1;
            if (source_changed) use_model <= !use_model;
        end
    end

    // -------------------------------------------------------------------------
    // Сценарии, хранилище, мастер и шина
    // -------------------------------------------------------------------------
    logic                seq_finished, seq_error;
    logic [ADDR_W-1:0]   info_addr, cursor;
    logic [15:0]         info_data;
    logic                info_empty;
    logic [7:0]          polls_last;
    logic                wr_p, rd_p, err_p;

    logic                ring_cmd_valid, ring_cmd_ready, ring_done, ring_ok, ring_empty, ring_err, ring_busy, ring_ovf;
    logic [1:0]          ring_cmd_op;
    logic [15:0]         ring_cmd_data, ring_data;
    logic [PTR_W:0]      ring_count;
    logic [PTR_W-1:0]    ring_wr_ptr, ring_rd_ptr;

    logic                raw_valid, raw_ready, raw_rd_valid, raw_rd_ready, raw_rd_last;
    logic                raw_done, raw_ok, raw_nack, raw_bus, raw_poll, polling;
    logic [2:0]          raw_op;
    logic [ADDR_W-1:0]   raw_addr;
    logic [15:0]         raw_data;
    logic [LEN_W-1:0]    raw_len;
    logic [7:0]          raw_rd_data, raw_polls;

    logic                cmd_valid, cmd_ready, cmd_read, cmd_stop;
    logic [6:0]          cmd_addr;
    logic [LEN_W-1:0]    cmd_len;
    logic [7:0]          wr_data, rd_data;
    logic                wr_valid, wr_ready, rd_valid, rd_ready, rd_last;
    logic                done, res_nack_addr, res_nack_data, res_timeout, res_arb;
    logic [LEN_W-1:0]    res_len;
    logic                ev_start, ev_stop, ev_ack, ev_nack;
    logic                model_busy;
    logic [15:0]         model_cycles;

    eeprom_seq #(
        .RECORDS (RECORDS),
        .ADDR_W  (ADDR_W),
        .LEN_W   (LEN_W)
    ) u_seq (
        .clk            (clk),
        .rst_n          (rst_n_s),
        .run            (run_p),
        .script         (script),
        .busy           (seq_busy),
        .finished       (seq_finished),
        .error          (seq_error),
        .info_addr      (info_addr),
        .info_data      (info_data),
        .info_empty     (info_empty),
        .cursor         (cursor),
        .polls_last     (polls_last),
        .wr_p           (wr_p),
        .rd_p           (rd_p),
        .err_p          (err_p),
        .ring_cmd_valid (ring_cmd_valid),
        .ring_cmd_ready (ring_cmd_ready),
        .ring_cmd_op    (ring_cmd_op),
        .ring_cmd_data  (ring_cmd_data),
        .ring_done      (ring_done),
        .ring_ok        (ring_ok),
        .ring_empty     (ring_empty),
        .ring_err       (ring_err),
        .ring_data      (ring_data),
        .ring_wr_ptr    (ring_wr_ptr),
        .ring_rd_ptr    (ring_rd_ptr),
        .raw_valid      (raw_valid),
        .raw_ready      (raw_ready),
        .raw_op         (raw_op),
        .raw_addr       (raw_addr),
        .raw_data       (raw_data),
        .raw_len        (raw_len),
        .raw_rd_valid   (raw_rd_valid),
        .raw_rd_data    (raw_rd_data),
        .raw_rd_ready   (raw_rd_ready),
        .raw_done       (raw_done),
        .raw_ok         (raw_ok),
        .raw_polls      (raw_polls)
    );

    eeprom_store #(
        .LEN_W      (LEN_W),
        .ADDR_W     (ADDR_W),
        .RECORDS    (RECORDS),
        .DEV_ADDR   (EE_ADDR),
        .PAGE_BYTES (16),
        .POLL_MAX   (POLL_MAX)
    ) u_store (
        .clk            (clk),
        .rst_n          (rst_n_s),
        .ring_cmd_valid (ring_cmd_valid),
        .ring_cmd_ready (ring_cmd_ready),
        .ring_cmd_op    (ring_cmd_op),
        .ring_cmd_data  (ring_cmd_data),
        .ring_done      (ring_done),
        .ring_ok        (ring_ok),
        .ring_empty     (ring_empty),
        .ring_err       (ring_err),
        .ring_data      (ring_data),
        .ring_busy      (ring_busy),
        .ring_ovf       (ring_ovf),
        .ring_count     (ring_count),
        .ring_wr_ptr    (ring_wr_ptr),
        .ring_rd_ptr    (ring_rd_ptr),
        .raw_valid      (raw_valid),
        .raw_ready      (raw_ready),
        .raw_op         (raw_op),
        .raw_addr       (raw_addr),
        .raw_data       (raw_data),
        .raw_len        (raw_len),
        .raw_rd_valid   (raw_rd_valid),
        .raw_rd_ready   (raw_rd_ready),
        .raw_rd_data    (raw_rd_data),
        .raw_rd_last    (raw_rd_last),
        .raw_done       (raw_done),
        .raw_ok         (raw_ok),
        .raw_nack       (raw_nack),
        .raw_bus        (raw_bus),
        .raw_poll       (raw_poll),
        .raw_polls      (raw_polls),
        .polling        (polling),
        .cmd_valid      (cmd_valid),
        .cmd_ready      (cmd_ready),
        .cmd_addr       (cmd_addr),
        .cmd_read       (cmd_read),
        .cmd_len        (cmd_len),
        .cmd_stop       (cmd_stop),
        .wr_data        (wr_data),
        .wr_valid       (wr_valid),
        .wr_ready       (wr_ready),
        .rd_data        (rd_data),
        .rd_valid       (rd_valid),
        .rd_ready       (rd_ready),
        .rd_last        (rd_last),
        .done           (done),
        .res_nack_addr  (res_nack_addr),
        .res_nack_data  (res_nack_data),
        .res_timeout    (res_timeout),
        .res_arb        (res_arb)
    );

    eeprom_bus #(
        .CLK_FREQ       (CLK_FREQ),
        .SCL_FREQ       (SCL_FREQ),
        .TIMEOUT_US     (TIMEOUT_US),
        .LEN_W          (LEN_W),
        .MODEL_ADDR     (EE_ADDR),
        .MODEL_WRITE_US (MODEL_WRITE_US)
    ) u_bus (
        .clk           (clk),
        .rst_n         (rst_n_s),
        .use_model     (use_model),
        .model_busy    (model_busy),
        .model_cycles  (model_cycles),
        .cmd_valid     (cmd_valid),
        .cmd_ready     (cmd_ready),
        .cmd_addr      (cmd_addr),
        .cmd_read      (cmd_read),
        .cmd_len       (cmd_len),
        .cmd_stop      (cmd_stop),
        .wr_data       (wr_data),
        .wr_valid      (wr_valid),
        .wr_ready      (wr_ready),
        .rd_data       (rd_data),
        .rd_valid      (rd_valid),
        .rd_ready      (rd_ready),
        .rd_last       (rd_last),
        .done          (done),
        .res_len       (res_len),
        .res_nack_addr (res_nack_addr),
        .res_nack_data (res_nack_data),
        .res_timeout   (res_timeout),
        .res_arb       (res_arb),
        .ev_start      (ev_start),
        .ev_stop       (ev_stop),
        .ev_ack        (ev_ack),
        .ev_nack       (ev_nack),
        .i2c_scl       (i2c_scl),
        .i2c_sda       (i2c_sda)
    );

    // -------------------------------------------------------------------------
    // Индикация
    // -------------------------------------------------------------------------
    eeprom_ind #(
        .CLK_FREQ   (CLK_FREQ),
        .ADDR_W     (ADDR_W),
        .COUNT_W    (PTR_W + 1),
        .PTR_W      (PTR_W),
        .AUTO_CLKS  (AUTO_CLKS),
        .DIGIT_HZ   (DIGIT_HZ),
        .BLANK_CLKS (BLANK_CLKS),
        .FLASH_CLKS (FLASH_CLKS),
        .SEG_ACTIVE (SEG_ACTIVE),
        .DIG_ACTIVE (DIG_ACTIVE),
        .LED_ACTIVE (LED_ACTIVE)
    ) u_ind (
        .clk            (clk),
        .rst_n          (rst_n_s),
        .info_addr      (info_addr),
        .info_data      (info_data),
        .info_empty     (info_empty),
        .cursor         (cursor),
        .ring_count     (ring_count),
        .ring_wr_ptr    (ring_wr_ptr),
        .ring_rd_ptr    (ring_rd_ptr),
        .ring_ovf       (ring_ovf),
        .polls_last     (polls_last),
        .script         (script),
        .wr_p           (wr_p),
        .rd_p           (rd_p),
        .err_p          (err_p),
        .polling        (polling),
        .next_mode      (key_press[2]),
        .script_changed (script_changed),
        .run            (run_p),
        .clear          (source_changed),
        .seg            (seg),
        .dig            (dig),
        .led            (led)
    );

endmodule
