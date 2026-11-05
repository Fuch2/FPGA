// =============================================================================
// Проверка I2C на плате B: мастер на шине с датчиком LM75A и EEPROM AT24C08
//
// Цепочка: i2c_seq (сценарий) -> i2c_master -> шина I2C (SCL PIN_99, SDA PIN_98);
// результаты: i2c_res_parser -> i2c_ind (светодиоды и индикатор).
//
// Кнопки (после подавления дребезга, по нажатию):
//   KEY1 - выполнить выбранный сценарий;  KEY2 - следующий сценарий (0..5);
//   KEY3 - следующий режим индикатора;     KEY4 - обнулить счётчики и погасить led[3].
// Сценарии, режимы индикатора и светодиоды - в i2c_seq.sv и i2c_ind.sv:
//   сценарий: 0 SCAN, 1 DT_TEMP, 2 DT_TOS, 3 EE_READ, 4 EE_WRITE, 5 ABSENT;
//   led[0] START, led[1] ACK ведомого, led[2] NACK ведомого, led[3] была ошибка.
// =============================================================================
module I2C_topB #(
    parameter int         CLK_FREQ     = 50_000_000,
    parameter int         SCL_FREQ     = 100_000,
    parameter int         TIMEOUT_US   = 1_000,
    parameter logic [6:0] DT_ADDR      = 7'h48,
    parameter logic [6:0] EE_ADDR      = 7'h50,
    parameter int         DEB_CLKS     = CLK_FREQ / 1000 * 10,   // антидребезг 10 мс
    parameter int         DIGIT_HZ     = 1_000,
    parameter int         BLANK_CLKS   = 100,
    parameter int         FLASH_CLKS   = CLK_FREQ / 20,          // вспышка светодиода 50 мс
    parameter int         SCROLL_CLKS  = CLK_FREQ,               // показ одного найденного адреса 1 с
    parameter bit         SEG_ACTIVE   = 1'b0,
    parameter bit         DIG_ACTIVE   = 1'b0,
    parameter bit         LED_ACTIVE   = 1'b0
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

    localparam int N_SCRIPTS = 6;
    localparam int LEN_W     = 8;
    localparam int N_FOUND   = 8;

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

    // выбранный сценарий
    logic [2:0] script;

    always_ff @(posedge clk or negedge rst_n_s) begin
        if (!rst_n_s)             script <= '0;
        else if (key_press[1])    script <= (script == N_SCRIPTS - 1) ? 3'd0 : script + 1'b1;
    end

    // -------------------------------------------------------------------------
    // Мастер, сценарий, парсер
    // -------------------------------------------------------------------------
    logic             cmd_valid, cmd_ready, cmd_read, cmd_stop;
    logic [6:0]       cmd_addr;
    logic [LEN_W-1:0] cmd_len;
    logic [7:0]       wr_data, rd_data;
    logic             wr_valid, wr_ready, rd_valid, rd_ready, rd_last;
    logic             done, res_nack_addr, res_nack_data, res_timeout, res_arb;
    logic [LEN_W-1:0] res_len;
    logic             ev_start, ev_stop, ev_ack, ev_nack;

    i2c_master #(
        .CLK_FREQ   (CLK_FREQ),
        .SCL_FREQ   (SCL_FREQ),
        .TIMEOUT_US (TIMEOUT_US),
        .LEN_W      (LEN_W)
    ) u_i2c (
        .clk           (clk),
        .rst_n         (rst_n_s),
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

    logic seq_busy, seq_finished, seq_error, scan_active, expect_nack;

    i2c_seq #(
        .LEN_W   (LEN_W),
        .DT_ADDR (DT_ADDR),
        .EE_ADDR (EE_ADDR)
    ) u_seq (
        .clk           (clk),
        .rst_n         (rst_n_s),
        .start         (key_press[0]),
        .script        (script),
        .busy          (seq_busy),
        .finished      (seq_finished),
        .error         (seq_error),
        .scan_active   (scan_active),
        .expect_nack   (expect_nack),
        .cmd_valid     (cmd_valid),
        .cmd_ready     (cmd_ready),
        .cmd_addr      (cmd_addr),
        .cmd_read      (cmd_read),
        .cmd_len       (cmd_len),
        .cmd_stop      (cmd_stop),
        .wr_data       (wr_data),
        .wr_valid      (wr_valid),
        .wr_ready      (wr_ready),
        .done          (done),
        .res_nack_addr (res_nack_addr),
        .res_nack_data (res_nack_data),
        .res_timeout   (res_timeout),
        .res_arb       (res_arb)
    );

    logic [15:0]               rd_word;
    logic [6:0]                last_addr, found_addr;
    logic [7:0]                last_status;
    logic                      txn_p, nack_p, err_p;
    logic [$clog2(N_FOUND)-1:0] found_idx;
    logic [$clog2(N_FOUND):0]   found_cnt;

    // новое сканирование начинает список найденных заново
    wire scan_start = key_press[0] && !seq_busy && (script == 3'd0);

    i2c_res_parser #(
        .N_FOUND (N_FOUND)
    ) u_parse (
        .clk           (clk),
        .rst_n         (rst_n_s),
        .cmd_fire      (cmd_valid && cmd_ready),
        .cmd_addr      (cmd_addr),
        .cmd_read      (cmd_read),
        .done          (done),
        .res_nack_addr (res_nack_addr),
        .res_nack_data (res_nack_data),
        .res_timeout   (res_timeout),
        .res_arb       (res_arb),
        .expect_nack   (expect_nack),
        .rd_data       (rd_data),
        .rd_valid      (rd_valid),
        .rd_ready      (rd_ready),
        .found_en      (scan_active),
        .found_clear   (scan_start),
        .clear         (key_press[3]),
        .rd_word       (rd_word),
        .last_addr     (last_addr),
        .last_status   (last_status),
        .txn_p         (txn_p),
        .nack_p        (nack_p),
        .err_p         (err_p),
        .found_idx     (found_idx),
        .found_addr    (found_addr),
        .found_cnt     (found_cnt)
    );

    // -------------------------------------------------------------------------
    // Индикация
    // -------------------------------------------------------------------------
    i2c_ind #(
        .CLK_FREQ    (CLK_FREQ),
        .DIGIT_HZ    (DIGIT_HZ),
        .BLANK_CLKS  (BLANK_CLKS),
        .FLASH_CLKS  (FLASH_CLKS),
        .SCROLL_CLKS (SCROLL_CLKS),
        .N_FOUND     (N_FOUND),
        .SEG_ACTIVE  (SEG_ACTIVE),
        .DIG_ACTIVE  (DIG_ACTIVE),
        .LED_ACTIVE  (LED_ACTIVE)
    ) u_ind (
        .clk            (clk),
        .rst_n          (rst_n_s),
        .ev_start       (ev_start),
        .ev_ack         (ev_ack),
        .ev_nack        (ev_nack),
        .txn_p          (txn_p),
        .nack_p         (nack_p),
        .err_p          (err_p),
        .info_word      (rd_word),
        .info_addr      (last_addr),
        .info_status    (last_status),
        .found_cnt      (found_cnt),
        .found_addr     (found_addr),
        .found_idx      (found_idx),
        .script         (script),
        .run            (key_press[0] && !seq_busy),
        .run_scan       (script == 3'd0),
        .script_changed (key_press[1]),
        .clear          (key_press[3]),
        .next_mode      (key_press[2]),
        .seg            (seg),
        .dig            (dig),
        .led            (led)
    );

endmodule
