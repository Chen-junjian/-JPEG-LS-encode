module sync_fifo(
    fifo_wr ,
    fifo_rd ,
    fifo_din,
    fifo_do ,
    fifo_ful,
    fifo_empty,
    clk     ,
    rstn      
);

//-- FIFO_DEPTH:  must be 2^N, or there is bug;
//-- FIFO_ADDR_BIT: equal N
parameter   FIFO_WIDTH = 16, FIFO_DEPTH = 8, FIFO_ADDR_BIT = 3;

input   wire            fifo_wr     ;
input   wire            fifo_rd     ;
input   wire    [FIFO_WIDTH-1 : 0]  fifo_din;
output  reg     [FIFO_WIDTH-1 : 0]  fifo_do;//combination output
output  wire            fifo_ful    ;       //high active; combination output
output  wire            fifo_empty  ;       //high active; combination output
input   wire            clk, rstn   ;

reg     [FIFO_WIDTH-1 : 0]  mem [0: FIFO_DEPTH-1];
reg     [FIFO_ADDR_BIT :0]  rd_ptr;
reg     [FIFO_ADDR_BIT :0]  wr_ptr;


always @(posedge clk or negedge rstn)
if(~rstn) begin
    wr_ptr  <= 'd0;
end else if(fifo_wr) begin
    wr_ptr  <= wr_ptr + 1'b1;
end

always @(posedge clk or negedge rstn)
if(~rstn) begin
    rd_ptr  <= 'd0;
end else if(fifo_rd) begin
    rd_ptr  <= rd_ptr + 1'b1;
end

//-- write in
always @(posedge clk)   // or negedge rstn)
if(fifo_wr) begin
    mem[wr_ptr[FIFO_ADDR_BIT-1 :0]] <= fifo_din;
end

assign  fifo_ful = (wr_ptr[FIFO_ADDR_BIT] != rd_ptr[FIFO_ADDR_BIT])
                 & (wr_ptr[FIFO_ADDR_BIT-1 : 0] == rd_ptr[FIFO_ADDR_BIT-1 : 0]);

//-- read out
always @(*) begin
    fifo_do = mem[rd_ptr[FIFO_ADDR_BIT-1 : 0]];
end


assign  fifo_empty = (wr_ptr == rd_ptr)? 1'b1 : 1'b0;

endmodule

