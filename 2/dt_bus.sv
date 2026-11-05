// =============================================================================
// I2C-мастер с выбором шины: реальные выводы с датчиком или внутренний имитатор ДТ
//
// Обёртка: i2c_txn + i2c_bit (то же, что i2c_master) + dt_model. Вход use_model:
//   0 - мастер подключён к выводам i2c_scl / i2c_sda (на плате там LM75A);
//   1 - выводы отпущены, мастер работает со шиной внутри ПЛИС, на которой висит
//       только имитатор dt_model: проверка чтения и индикации без датчика.
// Внутренняя шина - «проводное И»: линия в 0, если её прижимает хоть один участник.
// Пользовательская сторона (команды, потоки, итог, события) - как у i2c_master,
// подробности в i2c_txn.sv. gen_* - управление имитатором (см. dt_model.sv).
// Переключать use_model лучше между транзакциями.
// =============================================================================
module dt_bus #(
    parameter int         CLK_FREQ        = 50_000_000,  // частота clk, Гц
    parameter int         SCL_FREQ        = 100_000,     // частота SCL, Гц
    parameter int         TIMEOUT_US      = 1_000,       // предельное удержание SCL ведомым, мкс
    parameter int         LEN_W           = 8,
    parameter logic [6:0] MODEL_ADDR      = 7'h48,       // адрес имитатора
    parameter int         MODEL_UPDATE_MS = 100          // период обновления температуры имитатора, мс
) (
    input  logic               clk,
    input  logic               rst_n,           // асинхронный сброс, активный 0

    input  logic               use_model,

    // управление имитатором
    input  logic               gen_restart,
    input  logic [1:0]         gen_mode,
    input  logic signed [10:0] gen_lo,
    input  logic signed [10:0] gen_hi,
    input  logic        [10:0] gen_step,
    input  logic        [7:0]  gen_noise_mask,

    // команда
    input  logic               cmd_valid,
    output logic               cmd_ready,
    input  logic [6:0]         cmd_addr,
    input  logic               cmd_read,
    input  logic [LEN_W-1:0]   cmd_len,
    input  logic               cmd_stop,

    // поток записи
    input  logic [7:0]         wr_data,
    input  logic               wr_valid,
    output logic               wr_ready,

    // поток чтения
    output logic [7:0]         rd_data,
    output logic               rd_valid,
    input  logic               rd_ready,
    output logic               rd_last,

    // итог
    output logic               done,
    output logic [LEN_W-1:0]   res_len,
    output logic               res_nack_addr,
    output logic               res_nack_data,
    output logic               res_timeout,
    output logic               res_arb,

    // события
    output logic               ev_start,
    output logic               ev_stop,
    output logic               ev_ack,
    output logic               ev_nack,

    // выводы шины
    inout  wire                i2c_scl,
    inout  wire                i2c_sda
);

    logic [1:0] bit_cmd;
    logic       bit_val, bit_valid, bit_ready;
    logic       bit_rsp_valid, bit_rsp_bit, bit_rsp_timeout, bit_rsp_arb;
    logic       scl_low, sda_low;           // что прижимает мастер
    logic       model_sda_low;              // что прижимает имитатор
    logic       scl_bus, sda_bus;           // уровни шины, которые видят мастер и имитатор

    i2c_txn #(
        .LEN_W (LEN_W)
    ) u_txn (
        .clk             (clk),
        .rst_n           (rst_n),
        .cmd_valid       (cmd_valid),
        .cmd_ready       (cmd_ready),
        .cmd_addr        (cmd_addr),
        .cmd_read        (cmd_read),
        .cmd_len         (cmd_len),
        .cmd_stop        (cmd_stop),
        .wr_data         (wr_data),
        .wr_valid        (wr_valid),
        .wr_ready        (wr_ready),
        .rd_data         (rd_data),
        .rd_valid        (rd_valid),
        .rd_ready        (rd_ready),
        .rd_last         (rd_last),
        .done            (done),
        .res_len         (res_len),
        .res_nack_addr   (res_nack_addr),
        .res_nack_data   (res_nack_data),
        .res_timeout     (res_timeout),
        .res_arb         (res_arb),
        .ev_start        (ev_start),
        .ev_stop         (ev_stop),
        .ev_ack          (ev_ack),
        .ev_nack         (ev_nack),
        .bit_cmd         (bit_cmd),
        .bit_val         (bit_val),
        .bit_valid       (bit_valid),
        .bit_ready       (bit_ready),
        .bit_rsp_valid   (bit_rsp_valid),
        .bit_rsp_bit     (bit_rsp_bit),
        .bit_rsp_timeout (bit_rsp_timeout),
        .bit_rsp_arb     (bit_rsp_arb)
    );

    i2c_bit #(
        .CLK_FREQ   (CLK_FREQ),
        .SCL_FREQ   (SCL_FREQ),
        .TIMEOUT_US (TIMEOUT_US)
    ) u_bit (
        .clk         (clk),
        .rst_n       (rst_n),
        .cmd         (bit_cmd),
        .cmd_bit     (bit_val),
        .cmd_valid   (bit_valid),
        .cmd_ready   (bit_ready),
        .rsp_valid   (bit_rsp_valid),
        .rsp_bit     (bit_rsp_bit),
        .rsp_timeout (bit_rsp_timeout),
        .rsp_arb     (bit_rsp_arb),
        .scl_in      (scl_bus),
        .sda_in      (sda_bus),
        .scl_low     (scl_low),
        .sda_low     (sda_low)
    );

    dt_model #(
        .CLK_FREQ  (CLK_FREQ),
        .ADDR      (MODEL_ADDR),
        .UPDATE_MS (MODEL_UPDATE_MS)
    ) u_model (
        .clk            (clk),
        .rst_n          (rst_n),
        .present        (use_model),
        .gen_restart    (gen_restart),
        .gen_mode       (gen_mode),
        .gen_lo         (gen_lo),
        .gen_hi         (gen_hi),
        .gen_step       (gen_step),
        .gen_noise_mask (gen_noise_mask),
        .scl_in         (scl_bus),
        .sda_in         (sda_bus),
        .sda_low        (model_sda_low)
    );

    // шина: внутренняя («проводное И») или выводы ПЛИС (открытый сток, только 0 или Z)
    assign scl_bus = use_model ? !scl_low : i2c_scl;
    assign sda_bus = use_model ? !(sda_low || model_sda_low) : i2c_sda;

    assign i2c_scl = (!use_model && scl_low) ? 1'b0 : 1'bz;
    assign i2c_sda = (!use_model && sda_low) ? 1'b0 : 1'bz;

endmodule
