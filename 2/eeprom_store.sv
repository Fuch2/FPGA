// =============================================================================
// Хранилище измерений в EEPROM: кольцевой буфер и прямой доступ к ячейкам
//
// Обёртка: eeprom_ring (указатели и очередь записей) + eeprom_ctrl (запись с
// ACK-polling, чтение) поверх i2c_master. Два способа работы с памятью:
//   1. кольцо - команды ring_* (START, PUSH, POP), см. eeprom_ring.sv;
//   2. прямой доступ - запросы raw_* (запись и чтение байтов, слов, последовательное
//      чтение по адресу), см. eeprom_ctrl.sv.
// Одновременно можно пользоваться только одним способом: пока кольцо выполняет
// команду (ring_busy = 1), прямые запросы не принимаются (raw_ready = 0).
// Итог прямой операции - raw_done (импульс) и флаги raw_ok, raw_nack, raw_bus,
// raw_poll, raw_polls; итог команды кольца - ring_done и флаги ring_ok, ring_empty,
// ring_err. Поток чтения прямого доступа - raw_data_*. polling = 1, пока идёт ожидание
// конца цикла записи (микросхема занята).
// Интерфейс к i2c_master повторяет порты i2c_master (как у dt_reader).
// =============================================================================
module eeprom_store #(
    parameter int         LEN_W      = 8,
    parameter int         ADDR_W     = 10,                      // разрядность адреса ячейки (AT24C08: 10 бит)
    parameter int         RECORDS    = 512,                     // записей в кольце
    parameter logic [6:0] DEV_ADDR   = 7'h50,                   // адрес блока 0 микросхемы
    parameter int         PAGE_BYTES = 16,
    parameter int         POLL_MAX   = 100
) (
    input  logic                       clk,
    input  logic                       rst_n,       // асинхронный сброс, активный 0

    // кольцевой буфер
    input  logic                       ring_cmd_valid,
    output logic                       ring_cmd_ready,
    input  logic [1:0]                 ring_cmd_op,
    input  logic [15:0]                ring_cmd_data,
    output logic                       ring_done,
    output logic                       ring_ok,
    output logic                       ring_empty,
    output logic                       ring_err,
    output logic [15:0]                ring_data,
    output logic                       ring_busy,
    output logic                       ring_ovf,
    output logic [$clog2(RECORDS):0]   ring_count,
    output logic [$clog2(RECORDS)-1:0] ring_wr_ptr,
    output logic [$clog2(RECORDS)-1:0] ring_rd_ptr,

    // прямой доступ
    input  logic                       raw_valid,
    output logic                       raw_ready,
    input  logic [2:0]                 raw_op,
    input  logic [ADDR_W-1:0]          raw_addr,
    input  logic [15:0]                raw_data,
    input  logic [LEN_W-1:0]           raw_len,
    output logic                       raw_rd_valid,
    input  logic                       raw_rd_ready,
    output logic [7:0]                 raw_rd_data,
    output logic                       raw_rd_last,
    output logic                       raw_done,
    output logic                       raw_ok,
    output logic                       raw_nack,
    output logic                       raw_bus,
    output logic                       raw_poll,
    output logic [7:0]                 raw_polls,
    output logic                       polling,

    // команды и поток записи для i2c_master
    output logic                       cmd_valid,
    input  logic                       cmd_ready,
    output logic [6:0]                 cmd_addr,
    output logic                       cmd_read,
    output logic [LEN_W-1:0]           cmd_len,
    output logic                       cmd_stop,
    output logic [7:0]                 wr_data,
    output logic                       wr_valid,
    input  logic                       wr_ready,

    // поток чтения
    input  logic [7:0]                 rd_data,
    input  logic                       rd_valid,
    output logic                       rd_ready,
    input  logic                       rd_last,

    // итог транзакции
    input  logic                       done,
    input  logic                       res_nack_addr,
    input  logic                       res_nack_data,
    input  logic                       res_timeout,
    input  logic                       res_arb
);

    // запрос от кольца
    logic              rg_req_valid, rg_req_ready, rg_data_ready;
    logic [2:0]        rg_req_op;
    logic [ADDR_W-1:0] rg_req_addr;
    logic [15:0]       rg_req_data;
    logic [LEN_W-1:0]  rg_req_len;

    // запрос, выбранный для eeprom_ctrl
    logic              req_valid, req_ready, data_valid, data_ready, data_last, op_done, op_ok;
    logic [2:0]        req_op;
    logic [ADDR_W-1:0] req_addr;
    logic [15:0]       req_data;
    logic [LEN_W-1:0]  req_len;
    logic [7:0]        data;

    // пока кольцо занято, к eeprom_ctrl подключено оно
    assign req_valid  = ring_busy ? rg_req_valid : raw_valid;
    assign req_op     = ring_busy ? rg_req_op    : raw_op;
    assign req_addr   = ring_busy ? rg_req_addr  : raw_addr;
    assign req_data   = ring_busy ? rg_req_data  : raw_data;
    assign req_len    = ring_busy ? rg_req_len   : raw_len;
    assign data_ready = ring_busy ? rg_data_ready : raw_rd_ready;

    assign rg_req_ready = req_ready;
    assign raw_ready    = req_ready && !ring_busy;

    assign raw_rd_valid = data_valid && !ring_busy;
    assign raw_rd_data  = data;
    assign raw_rd_last  = data_last;
    assign raw_done     = op_done && !ring_busy;

    eeprom_ring #(
        .RECORDS (RECORDS),
        .ADDR_W  (ADDR_W),
        .LEN_W   (LEN_W)
    ) u_ring (
        .clk        (clk),
        .rst_n      (rst_n),
        .cmd_valid  (ring_cmd_valid),
        .cmd_ready  (ring_cmd_ready),
        .cmd_op     (ring_cmd_op),
        .cmd_data   (ring_cmd_data),
        .done       (ring_done),
        .res_ok     (ring_ok),
        .res_empty  (ring_empty),
        .res_err    (ring_err),
        .res_data   (ring_data),
        .busy       (ring_busy),
        .ovf        (ring_ovf),
        .count      (ring_count),
        .wr_ptr     (ring_wr_ptr),
        .rd_ptr     (ring_rd_ptr),
        .req_valid  (rg_req_valid),
        .req_ready  (rg_req_ready),
        .req_op     (rg_req_op),
        .req_addr   (rg_req_addr),
        .req_data   (rg_req_data),
        .req_len    (rg_req_len),
        .data_valid (data_valid && ring_busy),
        .data_ready (rg_data_ready),
        .data       (data),
        .op_done    (op_done),
        .op_ok      (op_ok)
    );

    eeprom_ctrl #(
        .LEN_W      (LEN_W),
        .ADDR_W     (ADDR_W),
        .DEV_ADDR   (DEV_ADDR),
        .PAGE_BYTES (PAGE_BYTES),
        .POLL_MAX   (POLL_MAX)
    ) u_ctrl (
        .clk           (clk),
        .rst_n         (rst_n),
        .req_valid     (req_valid),
        .req_ready     (req_ready),
        .req_op        (req_op),
        .req_addr      (req_addr),
        .req_data      (req_data),
        .req_len       (req_len),
        .data_valid    (data_valid),
        .data_ready    (data_ready),
        .data          (data),
        .data_last     (data_last),
        .op_done       (op_done),
        .op_ok         (op_ok),
        .op_nack       (raw_nack),
        .op_bus        (raw_bus),
        .op_poll       (raw_poll),
        .op_polls      (raw_polls),
        .polling       (polling),
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
        .res_nack_addr (res_nack_addr),
        .res_nack_data (res_nack_data),
        .res_timeout   (res_timeout),
        .res_arb       (res_arb)
    );

    assign raw_ok = op_ok;

endmodule
