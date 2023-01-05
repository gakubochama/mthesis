
module m_top();
    reg r_clk=0; initial forever #50 r_clk = ~r_clk;
    reg r_rst=0;
    reg [3:0] r_sel = 5; //control ALU
    reg [31:0] r_rrs = 15;
    reg [31:0] r_rrt = 13;
    wire[31:0] w_rslt;

/*
    always@(posedge r_clk) begin
        #50 $write("%4d cycle\nshitregA:%b\nshitregB:%b\n",($time-50)/100,main0.r_shiftrega,main0.r_shiftregb);
    end
*/
    initial #8000 $write("%4d cycle\nrrs:%b\nrrt:%b\nresult:%b\n",($time-50)/100,r_rrs,r_rrt,w_rslt);

    m_main main0(r_clk,r_rst,r_sel,r_rrs,r_rrt,w_rslt);

    initial #8000 $finish;
endmodule


/*
module m_top();
    reg r_clk=0; initial forever #50 r_clk = ~r_clk;
    reg [3:0] r_a = 6;
    reg [3:0] r_b = 7;
    reg r_c = 0;

    always@(posedge r_clk) begin
        {r_c,r_a} <= {r_a + r_b + r_c,r_a[3:1]};
        r_b <= {1'b0,r_b[3:1]};
        $write("%4d cycle r_a:%b r_b:%b r_c:%b\n",($time-50)/100,r_a,r_b,r_c);
    end

    initial #500 $finish;
endmodule
*/

/*
module m_top();
    reg r_clk=0; initial forever #50 r_clk = ~r_clk;
    reg [31:0] r_shiftrega = 6;
    reg [31:0] r_shiftregb = 7;
    reg [5:0] r_cnt = 0;
    reg [1:0] r_carry = 0;
    //reg r_tmp = 0;
    reg r_state = 0;

    always@(posedge r_clk) begin 
        $write("%4d cycle\n shiftregA:%b\n shiftregB:%b\n",($time-50)/100,r_shiftrega,r_shiftregb);
    end

    always@(posedge r_clk) begin
        if(r_state==0) begin
            if(r_cnt[5]==0) begin
                //{r_carry,r_shiftrega[0]} <= r_shiftrega[0] + r_shiftregb[0] + r_carry;
                {r_carry,r_shiftrega} <= {r_shiftrega[0] + r_shiftregb[0] + r_carry,r_shiftrega[31:1]};
                r_shiftregb <= {1'b0,r_shiftregb[31:1]};
                r_cnt <= r_cnt + 1;
            end else begin
                r_state <= 1;
            end
        end
    end

    initial #3500 $finish;
endmodule
*/

