// =============================================================================
// I2C, нижний уровень: START, STOP, передача и приём одного бита
//
// Модуль управляет двумя линиями «открытого стока»: scl_low = 1 прижимает SCL
// к нулю, scl_low = 0 отпускает линию (подтяжка поднимает её в 1). Так же sda_low.
// Входы scl_in / sda_in - состояние линий на выводах (синхронизируются внутри).
// Если ведомый удерживает SCL в 0 (растяжка тактов), модуль ждёт; дольше
// TIMEOUT_US ждать нельзя: линии отпускаются, ответ rsp_timeout.
//
// Команда (valid/ready): cmd принимается в такте, где cmd_valid = 1 и cmd_ready = 1.
//   BIT_START (0) - START или повторный START; после него SCL = 0
//   BIT_STOP  (1) - STOP; после него обе линии отпущены (шина свободна)
//   BIT_WRITE (2) - выдать бит cmd_bit на SDA и один период SCL
//   BIT_READ  (3) - отпустить SDA и принять бит с линии
// Ответ: rsp_valid - импульс на 1 такт после выполнения команды;
//   rsp_bit     - бит, считанный с SDA в середине высокого уровня SCL (BIT_WRITE и BIT_READ);
//   rsp_timeout - SCL не поднялся за TIMEOUT_US;
//   rsp_arb     - на линии не то, что выставлено: при START SDA не поднялась
//                 (шина занята), при BIT_WRITE бит на SDA отличается от cmd_bit
//                 (проиграли арбитраж или линия закорочена).
// В обоих аварийных случаях обе линии отпускаются.
//
// Время шага - четверть периода SCL (QUARTER_CLKS тактов). При 100 кГц: низкий
// SCL и высокий SCL по 5 мкс; START держит SCL = 1 до спада SDA 5 мкс и SDA = 0
// до спада SCL 5 мкс; STOP: 5 мкс до подъёма SDA и 7.5 мкс свободной шины.
// Условия для Standard-mode (tLOW 4.7, tHIGH 4.0, tSU;STA 4.7, tHD;STA 4.0,
// tSU;STO 4.0, tBUF 4.7 мкс) выполняются с запасом.
// =============================================================================
module i2c_bit #(
    parameter int CLK_FREQ   = 50_000_000,  // частота clk, Гц
    parameter int SCL_FREQ   = 100_000,     // частота SCL, Гц
    parameter int TIMEOUT_US = 1_000        // предельное удержание SCL ведомым, мкс
) (
    input  logic       clk,
    input  logic       rst_n,       // асинхронный сброс, активный 0

    // команды
    input  logic [1:0] cmd,
    input  logic       cmd_bit,
    input  logic       cmd_valid,
    output logic       cmd_ready,

    // ответ
    output logic       rsp_valid,
    output logic       rsp_bit,
    output logic       rsp_timeout,
    output logic       rsp_arb,

    // линии шины
    input  logic       scl_in,
    input  logic       sda_in,
    output logic       scl_low,
    output logic       sda_low
);

    localparam logic [1:0] BIT_START = 2'd0,
                           BIT_STOP  = 2'd1,
                           BIT_WRITE = 2'd2,
                           BIT_READ  = 2'd3;

    localparam int QUARTER_CLKS = CLK_FREQ / (4 * SCL_FREQ);               // тактов на шаг
    localparam int TIMEOUT_CLKS = (CLK_FREQ / 1_000_000) * TIMEOUT_US;
    localparam int CNT_W        = $clog2(QUARTER_CLKS);
    localparam int TO_W         = $clog2(TIMEOUT_CLKS + 1);

    // IDLE - ждём команду; ACT - один такт на действие в начале шага;
    // HOLD - выдержка шага, пока SCL не удерживается ведомым
    typedef enum logic [1:0] {IDLE, ACT, HOLD} state_t;

    state_t           state;
    logic [1:0]       cmd_r;       // выполняемая команда
    logic             bit_r;       // бит для BIT_WRITE
    logic [2:0]       step;        // номер шага команды
    logic [CNT_W-1:0] cnt;         // такты внутри шага
    logic [TO_W-1:0]  stall;       // такты ожидания подъёма SCL
    logic [1:0]       scl_sync, sda_sync;

    wire scl_s = scl_sync[1];
    wire sda_s = sda_sync[1];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl_sync <= 2'b11;
            sda_sync <= 2'b11;
        end else begin
            scl_sync <= {scl_sync[0], scl_in};
            sda_sync <= {sda_sync[0], sda_in};
        end
    end

    wire is_bit     = (cmd_r == BIT_WRITE) || (cmd_r == BIT_READ);
    wire last_step  = is_bit ? (step == 3'd3) : (step == 3'd5);
    wire scl_free   = scl_low | scl_s;                                     // SCL не держит ведомый
    wire arb_start  = (cmd_r == BIT_START) && (step == 3'd0) && !sda_s;    // шина занята
    wire arb_write  = (cmd_r == BIT_WRITE) && (step == 3'd2) && (sda_s != bit_r);

    assign cmd_ready = (state == IDLE);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= IDLE;
            cmd_r       <= BIT_START;
            bit_r       <= 1'b0;
            step        <= '0;
            cnt         <= '0;
            stall       <= '0;
            scl_low     <= 1'b0;
            sda_low     <= 1'b0;
            rsp_valid   <= 1'b0;
            rsp_bit     <= 1'b0;
            rsp_timeout <= 1'b0;
            rsp_arb     <= 1'b0;
        end else begin
            rsp_valid <= 1'b0;

            case (state)
                IDLE: begin
                    if (cmd_valid) begin
                        cmd_r       <= cmd;
                        bit_r       <= cmd_bit;
                        step        <= '0;
                        rsp_timeout <= 1'b0;
                        rsp_arb     <= 1'b0;
                        state       <= ACT;
                    end
                end

                // действия в начале шага (линии меняются только здесь)
                ACT: begin
                    case (cmd_r)
                        BIT_START: case (step)
                            3'd0: sda_low <= 1'b0;      // SDA вверх (SCL = 0 или шина свободна)
                            3'd1: scl_low <= 1'b0;      // SCL вверх, выдержка tSU;STA
                            3'd3: sda_low <= 1'b1;      // START: SDA вниз при SCL = 1
                            3'd5: scl_low <= 1'b1;      // SCL вниз после tHD;STA
                            default: ;
                        endcase
                        BIT_STOP: case (step)
                            3'd0: sda_low <= 1'b1;      // SDA вниз при SCL = 0
                            3'd1: scl_low <= 1'b0;      // SCL вверх, выдержка tSU;STO
                            3'd3: sda_low <= 1'b0;      // STOP: SDA вверх при SCL = 1
                            default: ;                  // свободная шина tBUF
                        endcase
                        default: case (step)            // BIT_WRITE, BIT_READ
                            3'd0: sda_low <= (cmd_r == BIT_WRITE) && !bit_r;   // данные при SCL = 0
                            3'd1: scl_low <= 1'b0;      // SCL вверх
                            3'd3: scl_low <= 1'b1;      // SCL вниз
                            default: ;
                        endcase
                    endcase
                    cnt   <= '0;
                    stall <= '0;
                    state <= HOLD;
                end

                HOLD: begin
                    if (!scl_free) begin
                        // отпустили SCL, а он ещё (или уже ведомым) в 0
                        if (stall == TIMEOUT_CLKS - 1) begin
                            scl_low     <= 1'b0;
                            sda_low     <= 1'b0;
                            rsp_timeout <= 1'b1;
                            rsp_valid   <= 1'b1;
                            state       <= IDLE;
                        end else begin
                            stall <= stall + 1'b1;
                        end
                    end else if (cnt != QUARTER_CLKS - 2) begin
                        cnt <= cnt + 1'b1;
                    end else if (arb_start || arb_write) begin
                        scl_low   <= 1'b0;
                        sda_low   <= 1'b0;
                        rsp_arb   <= 1'b1;
                        rsp_valid <= 1'b1;
                        state     <= IDLE;
                    end else begin
                        if (is_bit && step == 3'd2) rsp_bit <= sda_s;      // выборка в середине SCL = 1
                        if (last_step) begin
                            rsp_valid <= 1'b1;
                            state     <= IDLE;
                        end else begin
                            step  <= step + 1'b1;
                            state <= ACT;
                        end
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
