// =============================================================================
// Имитатор ДТ: I2C-ведомый, ведущий себя как LM75A, температура задаётся программно
//
// Обёртка: i2c_slave (шина, байты) + temp_gen (последовательность температур)
// + регистры LM75A. Нужен, чтобы отлаживать dt_reader без реального датчика: в
// симуляции подключается к шине вместо датчика, а на плате - к внутренней шине
// внутри ПЛИС (см. DT_topB). Модуль синтезируемый.
//
// Регистры LM75A (указатель - первый байт после адреса при записи, 2 младших бита):
//   0  температура, только чтение, 2 байта (старший первым): 11 бит в D15..D5;
//   1  конфигурация, 1 байт, по умолчанию 0x00; бит 0 = 1 (shutdown) - температура
//      перестаёт обновляться;
//   2  Thyst, 2 байта, по умолчанию 0x4B00 (75 °C), значащие D15..D7;
//   3  Tos,   2 байта, по умолчанию 0x5000 (80 °C), значащие D15..D7.
// Чтение 16-битного регистра: байты чередуются старший / младший; указатель
// сохраняется между транзакциями. Значение температуры фиксируется в момент адреса
// чтения, поэтому два байта всегда принадлежат одному значению.
//
// Температура обновляется раз в UPDATE_MS миллисекунд (у LM75A - раз в ~100 мс)
// по правилу temp_gen: режим gen_mode (0 константа, 1 линейный рост, 2 пила, 3 шум),
// границы gen_lo, gen_hi и шаг gen_step в шагах 0.125 °C, gen_noise_mask - амплитуда
// шума, gen_restart (импульс) - начать последовательность заново.
// present = 0 - датчик «отключён», на адрес не отвечает (проверка NACK).
// =============================================================================
module dt_model #(
    parameter int         CLK_FREQ  = 50_000_000,   // частота clk, Гц
    parameter logic [6:0] ADDR      = 7'h48,        // адрес на шине
    parameter int         UPDATE_MS = 100           // период обновления температуры, мс
) (
    input  logic               clk,
    input  logic               rst_n,           // асинхронный сброс, активный 0

    input  logic               present,
    input  logic               gen_restart,
    input  logic [1:0]         gen_mode,
    input  logic signed [10:0] gen_lo,
    input  logic signed [10:0] gen_hi,
    input  logic        [10:0] gen_step,
    input  logic        [7:0]  gen_noise_mask,

    input  logic               scl_in,
    input  logic               sda_in,
    output logic               sda_low
);

    localparam int UPDATE_CLKS = CLK_FREQ / 1000 * UPDATE_MS;
    localparam int TICK_W      = $clog2(UPDATE_CLKS + 1);

    localparam logic [1:0] REG_TEMP = 2'd0,
                           REG_CONF = 2'd1,
                           REG_HYST = 2'd2,
                           REG_TOS  = 2'd3;

    // -------------------------------------------------------------------------
    // Шина: байтовый уровень
    // -------------------------------------------------------------------------
    logic       addr_hit, addr_read, rx_valid, tx_req;
    logic [7:0] rx_data, tx_data;

    i2c_slave #(
        .ADDR (ADDR)
    ) u_slave (
        .clk       (clk),
        .rst_n     (rst_n),
        .enable    (present),
        .addr_hit  (addr_hit),
        .addr_read (addr_read),
        .rx_valid  (rx_valid),
        .rx_data   (rx_data),
        .tx_req    (tx_req),
        .tx_data   (tx_data),
        .scl_in    (scl_in),
        .sda_in    (sda_in),
        .sda_low   (sda_low)
    );

    // -------------------------------------------------------------------------
    // Температура
    // -------------------------------------------------------------------------
    logic [15:0] temp_word;
    logic [TICK_W-1:0] tick_cnt;
    logic        tick;
    logic [7:0]  conf;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_cnt <= '0;
            tick     <= 1'b0;
        end else if (tick_cnt == UPDATE_CLKS - 1) begin
            tick_cnt <= '0;
            tick     <= !conf[0];               // shutdown: температура не обновляется
        end else begin
            tick_cnt <= tick_cnt + 1'b1;
            tick     <= 1'b0;
        end
    end

    temp_gen u_gen (
        .clk            (clk),
        .rst_n          (rst_n),
        .tick           (tick),
        .restart        (gen_restart),
        .gen_mode       (gen_mode),
        .gen_lo         (gen_lo),
        .gen_hi         (gen_hi),
        .gen_step       (gen_step),
        .gen_noise_mask (gen_noise_mask),
        .temp_val       (),
        .temp_word      (temp_word)
    );

    // -------------------------------------------------------------------------
    // Регистры
    // -------------------------------------------------------------------------
    logic [1:0]  ptr;
    logic [15:0] temp_snap, thyst, tos;
    logic [1:0]  wr_idx;            // сколько байт принято в этой записи (0 - указатель)
    logic        tx_idx;            // 0 - старший байт, 1 - младший
    logic [15:0] reg16;             // выбранный указателем 16-битный регистр

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ptr       <= REG_TEMP;
            conf      <= 8'h00;
            thyst     <= 16'h4B00;
            tos       <= 16'h5000;
            temp_snap <= '0;
            wr_idx    <= '0;
            tx_idx    <= 1'b0;
        end else begin
            if (addr_hit) begin
                wr_idx <= '0;
                tx_idx <= 1'b0;
                if (addr_read) temp_snap <= temp_word;
            end

            if (tx_req) tx_idx <= !tx_idx;

            if (rx_valid) begin
                if (wr_idx != 2'd3) wr_idx <= wr_idx + 1'b1;
                if (wr_idx == 2'd0) begin
                    ptr <= rx_data[1:0];
                end else begin
                    case (ptr)
                        REG_CONF: if (wr_idx == 2'd1) conf <= rx_data;
                        REG_HYST: if (wr_idx == 2'd1) thyst[15:8] <= rx_data;
                                  else if (wr_idx == 2'd2) thyst[7:0] <= rx_data & 8'h80;
                        REG_TOS:  if (wr_idx == 2'd1) tos[15:8] <= rx_data;
                                  else if (wr_idx == 2'd2) tos[7:0] <= rx_data & 8'h80;
                        default: ;
                    endcase
                end
            end
        end
    end

    always_comb begin
        case (ptr)
            REG_HYST: reg16 = thyst;
            REG_TOS:  reg16 = tos;
            default:  reg16 = temp_snap;
        endcase
        if (ptr == REG_CONF) tx_data = conf;
        else                 tx_data = tx_idx ? reg16[7:0] : reg16[15:8];
    end

endmodule
