`default_nettype none
`include "config.vh"

module testbench();
    reg clk = 0;
    initial forever #50 clk = !clk;

    reg rst_n = 0;
    initial #250 rst_n = 1;

    /* counter */
    reg [63:0] cnt = 0;
    always @(posedge clk) begin
        cnt  <= cnt + 1;
    end
    initial begin
        $dumpfile("main.vcd");
        $dumpvars(0,testbench);
    end

    wire       w_rxd = 1;
    wire       w_txd;
    wire [3:0] w_led;
    main main0(
        .w_clk(clk),
        .w_rst(!rst_n),
        .w_rxd(w_rxd),
        .r_txd(w_txd),
        .w_led(w_led)
    );

    /* tohost */
    wire       uartwe;
    wire [7:0] uartdata;
    serialc serial_ctrl0(
        .CLK  (clk),
        .RST_X(rst_n),
        .RXD  (w_txd),
        .EN   (uartwe),
        .DATA (uartdata)
    );
    /* Note!! */
    wire       uartwe_t   = (main0.p.w_ir==32'h00732023 & main0.p.r_state==`STORE_2);
    wire [7:0] uartdata_t = main0.p.regfile0.mem[main0.p.w_ir[24:20]][7:0];
    /**/
    always @(negedge clk) begin
//        if (uartwe) $write("%c", uartdata);
        if (uartwe_t) $write("%c", uartdata_t);
        if (main0.poweroff & (main0.queue_num==0) & main0.tx_ready & !main0.uartwe) begin
            $write("==> poweroff\n");
            $write("==> elapsed clock cycles       : %16d\n", cnt);
//            $write("==> valid instruction executed : %16d\n", icnt);
            $finish();
        end
    end

    /* time out */
`ifdef TIMEOUT
    always @(negedge clk) begin
        if (cnt>`TIMEOUT) begin
            $write("simulation time out!!\n");
            $finish();
        end
    end
`endif

    /* trace */
`ifdef TRACE
    integer fd_trace; initial fd_trace = $fopen(`TRACE_FILE, "w");
    reg [31:0] tc = 1;
    integer i, j;
    always @(negedge clk) begin
        if (!main0.r_core_rst && main0.p.r_valid && ((main0.p.r_state==`BRANCH_1 && main0.p.r_cnt[4]) || (main0.p.r_state==`WRITEBACK_1) || (main0.p.r_state==`STORE_2))) begin//((main0.p.r_state==`BRANCH_1 && main0.p.r_cnt[4]) || (main0.p.r_state==`WRITEBACK_1) || (main0.p.r_state==`STORE_2))) begin
            $fwrite(fd_trace, "%08d %08x %08x\n", tc, main0.p.r_vpc, main0.p.w_ir);
            for (i=0; i<4; i=i+1) begin
                for (j=0; j<8; j=j+1) begin
                    $fwrite(fd_trace, "%08x", ((i*8+j == main0.p.w_rd) && (i*8+j != 0)) ? main0.p.w_wdata : main0.p.regfile0.mem[i*8+j]);
                    $fwrite(fd_trace, "%c", ((j==7) ? "\n" : " "));
                end
            end
            tc <= tc + 1;
        end
    end 
`endif
endmodule

`default_nettype wire


//module testbench(
//    input wire clk,
//    input wire rst_n
//);

//    /* fromhost */
//    reg [31:0] mem [0:`MEMSIZE/4-1];
//    initial $readmemh(`IMEMFILE, mem);
//
//    reg   [1:0] bitcnt    = 0;
//    reg  [31:0] addr      = 0;
//    reg         init_we   = 0;
//    reg  [31:0] init_data = 0;
//    wire        init_done = ((addr>=`MEMSIZE/4) & (bitcnt==0));
//    wire        tx_ready;
//    wire        en        = (!init_done & tx_ready & !init_we);
//    always @(posedge clk) begin
//        bitcnt    <= (!rst_n) ? 0 : (en) ? bitcnt+1 : bitcnt;
//        addr      <= (!rst_n) ? 0 : (en & (bitcnt==0)) ? addr+1 : addr;
//        init_we   <= (!rst_n) ? 0 : (en) ? 1 : 0;
//        init_data <= (!rst_n) ? 0 : (en) ? ((bitcnt==0) ? mem[addr] : {8'h0, init_data[31:8]}) : init_data;
//    end
//
//    wire txd;
//    uart_tx uart_tx0(
//        .clk  (clk),
//        .rst_n(rst_n),
//        .we   (init_we),
//        .data (init_data[7:0]),
//        .ready(tx_ready),
//        .txd  (txd)
//    );
//
//    /* processor */
//    wire rxd;
//    proc proc0(
//        .i_clk  (clk),
//        .i_rst_n(rst_n),
//        .i_rxd  (txd),
//        .o_txd  (rxd)
//    );
//
//    /* instruction counter */
//    reg [63:0] icnt = 0;
//    always @(posedge clk) begin
//        icnt <= (!proc0.core_rst_n) ? 0 : (proc0.power_off) ? icnt : icnt+1;
//    end
//
//    /* tohost */
//    wire       uartwe;
//    wire [7:0] uartdata;
//    serial_ctrl serial_ctrl0(
//        .clk  (clk),
//        .rst_n(rst_n),
//        .rxd  (rxd),
//        .en   (uartwe),
//        .data (uartdata)
//    );
//    always @(posedge clk) begin
//        if (uartwe) $write("%c", uartdata);
//        if (proc0.power_off & (proc0.uart0.q_num==0) & proc0.uart0.tx_ready & !proc0.uart0.uartwe) begin
//            $write("\n==> power_off\n");
//            $write("==> elapsed clock cycles       : %16d\n", cnt);
//            $write("==> valid instruction executed : %16d\n", icnt);
//            $finish();
//        end
//    end
