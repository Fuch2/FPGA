// =============================================================================
// I2C-мастер: START, STOP, повторный START, запись и чтение байтов, ACK/NACK,
// обнаружение отсутствия ведомого (NACK на адрес) и тайм-аут растяжки SCL
//
// Обёртка: i2c_txn (транзакции: адрес, R/W, число байт, потоки данных)
// поверх i2c_bit (START, STOP, бит). Линии SCL и SDA - «открытый сток»:
// мастер только прижимает их к нулю или отпускает (Z), единицу выставляет
// внешняя подтяжка (на выводе ПЛИС - weak pull-up).
//
// Интерфейс (подробности - в i2c_txn.sv):
//   команда (valid/ready): cmd_addr[6:0], cmd_read, cmd_len, cmd_stop
//   запись (valid/ready):  wr_data - очередной байт, когда wr_ready = 1
//   чтение (valid/ready):  rd_data, rd_last - принятый байт, когда rd_valid = 1
//   итог: импульс done и флаги res_len, res_nack_addr, res_nack_data,
//         res_timeout, res_arb
//   события: ev_start, ev_stop, ev_ack, ev_nack (импульсы)
// =============================================================================
module i2c_master #(
    parameter int CLK_FREQ   = 50_000_000,  // частота clk, Гц
    parameter int SCL_FREQ   = 100_000,     // частота SCL, Гц
    parameter int TIMEOUT_US = 1_000,       // предельное удержание SCL ведомым, мкс
    parameter int LEN_W      = 8            // разрядность числа байт в команде
) (
    input  logic             clk,
    input  logic             rst_n,         // асинхронный сброс, активный 0

    // команда
    input  logic             cmd_valid,
    output logic             cmd_ready,
    input  logic [6:0]       cmd_addr,
    input  logic             cmd_read,
    input  logic [LEN_W-1:0] cmd_len,
    input  logic             cmd_stop,

    // поток записи
    input  logic [7:0]       wr_data,
    input  logic             wr_valid,
    output logic             wr_ready,

    // поток чтения
    output logic [7:0]       rd_data,
    output logic             rd_valid,
    input  logic             rd_ready,
    output logic             rd_last,

    // итог
    output logic             done,
    output logic [LEN_W-1:0] res_len,
    output logic             res_nack_addr,
    output logic             res_nack_data,
    output logic             res_timeout,
    output logic             res_arb,

    // события
    output logic             ev_start,
    output logic             ev_stop,
    output logic             ev_ack,
    output logic             ev_nack,

    // шина
    inout  wire              i2c_scl,
    inout  wire              i2c_sda
);

    logic [1:0] bit_cmd;
    logic       bit_val, bit_valid, bit_ready;
    logic       bit_rsp_valid, bit_rsp_bit, bit_rsp_timeout, bit_rsp_arb;
    logic       scl_low, sda_low;

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
        .scl_in      (i2c_scl),
        .sda_in      (i2c_sda),
        .scl_low     (scl_low),
        .sda_low     (sda_low)
    );

    // открытый сток: только 0 или Z
    assign i2c_scl = scl_low ? 1'b0 : 1'bz;
    assign i2c_sda = sda_low ? 1'b0 : 1'bz;

endmodule
