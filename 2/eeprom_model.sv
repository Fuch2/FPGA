// =============================================================================
// Имитатор EEPROM AT24C08: I2C-ведомый с памятью и внутренним циклом записи
//
// Синтезируемый модуль: в симуляции подключается к шине вместо микросхемы, на плате -
// к внутренней шине внутри ПЛИС (см. eeprom_bus). Ведёт себя как настоящая микросхема:
//   * адрес на шине 1010 A2 P1 P0: ADDR (блок 0), ADDR + 1 ... - блоки по 256 байт,
//     номер блока P1 P0 - старшие биты адреса ячейки (при MEM_BYTES = 1024 - 4 блока);
//   * запись: первый байт после адреса - младшие 8 бит адреса ячейки, остальные байты
//     попадают в буфер страницы (PAGE_BYTES байт); если байтов больше страницы или
//     адрес в конце страницы, адрес переворачивается внутри страницы и новые байты
//     затирают прежние;
//   * цикл записи начинается по STOP, если в буфер попал хотя бы один байт данных;
//     в это время (WRITE_US мкс, у AT24C08 до 5 мс) ведомый не отвечает ни на какой
//     адрес - ACK-polling видит NACK; запись без STOP (повторный START вместо него)
//     не выполняется; запись одного адреса без данных цикла не вызывает;
//   * чтение: с текущего адреса, адрес увеличивается после каждого байта по всей
//     памяти и по концу памяти переходит на 0; указатель сохраняется между
//     транзакциями («текущий адрес»). Чтение с произвольного адреса - запись адреса
//     без STOP, повторный START и чтение;
//   * после включения содержимое 0xFF (новая микросхема).
// Память mem[0 .. MEM_BYTES-1] доступна тестбенчу по иерархии (model.mem[i]); внутри
// ПЛИС она ложится в блочную память. Тактовая частота clk не меньше чем в 10 раз выше SCL.
//
// Входы-выходы:
//   present    - 0: микросхема «не припаяна», на адрес не отвечает;
//   busy       - идёт внутренний цикл записи;
//   wr_cycles  - сколько циклов записи выполнено (для проверки числа записей).
// =============================================================================
module eeprom_model #(
    parameter int         CLK_FREQ   = 50_000_000,   // частота clk, Гц
    parameter logic [6:0] ADDR       = 7'h50,        // адрес блока 0 (1010 A2 00, A2 = 0)
    parameter int         MEM_BYTES  = 1024,         // объём: 512, 1024 или 2048 байт
    parameter int         PAGE_BYTES = 16,           // размер страницы записи, степень двойки
    parameter int         WRITE_US   = 5_000         // длительность цикла записи, мкс (AT24C08: до 5 мс)
) (
    input  logic        clk,
    input  logic        rst_n,          // асинхронный сброс, активный 0

    input  logic        present,
    output logic        busy,
    output logic [15:0] wr_cycles,

    input  logic        scl_in,
    input  logic        sda_in,
    output logic        sda_low
);

    localparam int ADDR_W     = $clog2(MEM_BYTES);
    localparam int BLOCKS     = MEM_BYTES / 256;
    localparam int BLK_W      = ADDR_W - 8;
    localparam int PAGE_W     = $clog2(PAGE_BYTES);
    localparam int WRITE_CLKS = CLK_FREQ / 1_000_000 * WRITE_US;
    localparam int CYCLE_W    = $clog2(WRITE_CLKS + 1);

    // -------------------------------------------------------------------------
    // Шина: по одному байтовому ведомому на каждый блок (адрес ADDR + номер блока)
    // -------------------------------------------------------------------------
    logic [BLOCKS-1:0] hit_v, read_v, rx_valid_v, tx_req_v, sda_low_v;
    logic [7:0]        rx_data_v [BLOCKS];
    logic [7:0]        tx_data;

    genvar b;   // Quartus Standard не понимает genvar внутри for

    generate
    for (b = 0; b < BLOCKS; b++) begin : g_slave
        i2c_slave #(
            .ADDR (ADDR + 7'(b))
        ) u_slave (
            .clk       (clk),
            .rst_n     (rst_n),
            .enable    (present && !busy),
            .addr_hit  (hit_v[b]),
            .addr_read (read_v[b]),
            .rx_valid  (rx_valid_v[b]),
            .rx_data   (rx_data_v[b]),
            .tx_req    (tx_req_v[b]),
            .tx_data   (tx_data),
            .scl_in    (scl_in),
            .sda_in    (sda_in),
            .sda_low   (sda_low_v[b])
        );
    end
    endgenerate

    logic [BLK_W-1:0] blk_now;                  // блок, ответивший на адрес (в такте addr_hit)
    logic [BLK_W-1:0] blk_sel;                  // блок текущей транзакции
    logic [7:0]       rx_data;

    always_comb begin
        blk_now = '0;
        for (int i = 0; i < BLOCKS; i++)
            if (hit_v[i]) blk_now = BLK_W'(i);
    end

    wire addr_hit  = |hit_v;
    wire addr_read = read_v[blk_now];
    wire rx_valid  = |rx_valid_v;
    wire tx_req    = |tx_req_v;

    assign rx_data = rx_data_v[blk_sel];
    assign sda_low = |sda_low_v;

    // START и STOP (тот же конвейер синхронизации, что в i2c_slave)
    logic [2:0] scl_sync, sda_sync;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl_sync <= 3'b111;
            sda_sync <= 3'b111;
        end else begin
            scl_sync <= {scl_sync[1:0], scl_in};
            sda_sync <= {sda_sync[1:0], sda_in};
        end
    end

    wire start_ev = scl_sync[1] && sda_sync[2] && !sda_sync[1];
    wire stop_ev  = scl_sync[1] && !sda_sync[2] && sda_sync[1];

    // -------------------------------------------------------------------------
    // Память, указатель, буфер страницы, цикл записи
    // -------------------------------------------------------------------------
    logic [7:0]             mem [MEM_BYTES];
    logic [7:0]             mem_q;              // данные по указателю ptr (чтение с задержкой в такт)
    (* ramstyle = "logic" *) logic [7:0] page_data [PAGE_BYTES];   // 16 байт - в регистрах, не в блочной памяти
    logic [PAGE_BYTES-1:0]  page_valid;         // в какие байты буфера страницы приходили данные
    logic [ADDR_W-1:0]      ptr;                // текущий адрес
    logic                   first_byte;         // следующий принятый байт - адрес ячейки
    logic                   committing;         // буфер страницы переписывается в память
    logic [PAGE_W-1:0]      commit_idx;
    logic [ADDR_W-PAGE_W-1:0] commit_page;
    logic [CYCLE_W-1:0]     cycle_cnt;          // сколько тактов осталось до конца цикла записи

    initial begin
        for (int i = 0; i < MEM_BYTES; i++) mem[i] = 8'hFF;      // новая микросхема
    end

    wire                mem_we    = committing && page_valid[commit_idx];
    wire [ADDR_W-1:0]   mem_waddr = {commit_page, commit_idx};

    // данные для чтения готовы через такт после смены ptr; следующий байт нужен не раньше
    // чем через байт SCL, так что запаса достаточно. Обычный always, а не always_ff:
    // память инициализируется в initial и доступна тестбенчу по иерархии
    always @(posedge clk) begin
        if (mem_we) mem[mem_waddr] <= page_data[commit_idx];
        mem_q <= mem[ptr];
    end

    always_ff @(posedge clk) begin
        if (rx_valid && !first_byte) page_data[ptr[PAGE_W-1:0]] <= rx_data;
    end

    assign tx_data = mem_q;
    assign busy    = (cycle_cnt != '0);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            blk_sel     <= '0;
            ptr         <= '0;
            first_byte  <= 1'b0;
            page_valid  <= '0;
            committing  <= 1'b0;
            commit_idx  <= '0;
            commit_page <= '0;
            cycle_cnt   <= '0;
            wr_cycles   <= '0;
        end else begin
            if (cycle_cnt != '0) cycle_cnt <= cycle_cnt - 1'b1;

            // перенос буфера страницы в память: по байту за такт, внутри цикла записи
            if (committing) begin
                commit_idx <= commit_idx + 1'b1;
                if (commit_idx == PAGE_W'(PAGE_BYTES - 1)) begin
                    committing <= 1'b0;
                    page_valid <= '0;
                end
            end

            if (start_ev && !committing) page_valid <= '0;       // запись без STOP отменяется

            if (addr_hit) begin
                blk_sel            <= blk_now;
                ptr[ADDR_W-1:8]    <= blk_now;
                first_byte         <= !addr_read;
            end

            if (rx_valid) begin
                if (first_byte) begin
                    ptr[7:0]   <= rx_data;
                    first_byte <= 1'b0;
                end else begin
                    page_valid[ptr[PAGE_W-1:0]] <= 1'b1;
                    ptr[PAGE_W-1:0]             <= ptr[PAGE_W-1:0] + 1'b1;   // внутри страницы
                end
            end

            if (tx_req) ptr <= ptr + 1'b1;

            if (stop_ev && !committing && (page_valid != '0)) begin
                committing  <= 1'b1;
                commit_idx  <= '0;
                commit_page <= ptr[ADDR_W-1:PAGE_W];
                cycle_cnt   <= CYCLE_W'(WRITE_CLKS);
                wr_cycles   <= wr_cycles + 1'b1;
            end
        end
    end

endmodule
