// Description  : 
// 1: receive pixel from input;
// 2: get a/b/c/d neighbour for gradient cal;
//----------------------------------------------------------------------------//

module jls_recv_pixel(
    //cfg info
    xsize           ,
    ysize           ,

    //image pixel recv
    sof             , 
    pixel_vld       ,
    pixel           ,
    pixel_ack       ,

    //top line mem r/w
    top_mem_ce      ,
    top_mem_we      ,
    top_mem_addr    ,
    top_mem_wdata   ,
    top_mem_rdata   ,

    //pipeline handshake with downstream
    s0_vld          ,
    s0_ack          ,
    s0_x            ,
    s0_y            ,
    s0_pixel        ,
    s0_xend         ,
    s0_xbeg         ,
    s0_a            ,
    s0_b            ,
    s0_c            ,
    s0_d            ,

    s1_pixel        ,

    clk             ,
    rstn         
);

parameter       xy_bits = 11;   //maximal 2048x2048 pixels


input   wire            clk, rstn       ;

//cfg info
input   wire    [xy_bits-1:0]  xsize    ;   //cnt from 0, frame x len should be 2*N pixels (odd pixel number)
input   wire    [xy_bits-1:0]  ysize    ;   //cnt from 0

//image pixel recv
input   wire            sof             ;   //1T pulse start a frame
input   wire            pixel_vld       ;
input   wire    [7:0]   pixel           ;   //单像素通道 8bit
output  wire            pixel_ack       ;

//top line mem r/w
output  wire            top_mem_ce      ;   //mem ce
output  reg             top_mem_we      ;
output  wire    [xy_bits-2:0]top_mem_addr;  //each addr store 2 pixels
output  wire    [15:0]  top_mem_wdata   ;
input   wire    [15:0]  top_mem_rdata   ;

//pipeline handshake with downstream
output  reg             s0_vld          ;   //handshake between s0 and s1
input   wire            s0_ack          ;
output  reg     [xy_bits-1:0]   s0_x    ;   //xloc within a frame, cnt from 0
output  reg     [xy_bits-1:0]   s0_y    ;   //yloc within a frame, cnt from 0
output  reg     [7:0]   s0_pixel        ;   //current pixel from JLS encoding
output  reg             s0_xend         ;   //1: last pixel of a line
output  reg             s0_xbeg         ;   //1: first pixel of a line
output  reg     [7:0]   s0_a            ;   //neighbour_a
output  reg     [7:0]   s0_b            ;   //neighbour_b
output  reg     [7:0]   s0_c            ;   //neighbour_c
output  reg     [7:0]   s0_d            ;   //neighbour_d

input   wire    [7:0]   s1_pixel        ;   //org pixel at pipe stage s1(when s1_vld=1)


//--- 1: recv pixel  维护当前pixel 的横坐标和纵坐标
wire                    recv_pixel      ;
reg     [xy_bits-1:0]   in_x, in_y      ;   //recv pixel x/y loc, cnt from 0

assign  pixel_ack = (!s0_vld) | s0_ack;  // 流水线反压 当s0是空的，或者s0已经被下游接收了，才可以接收新的pixel
assign  recv_pixel= pixel_vld & pixel_ack;

always @(posedge clk)   // or negedge rstn)
if(sof)
    in_x    <= 'd0;  
else if(recv_pixel) begin
    if(in_x == xsize)
        in_x    <= 'd0;
    else
        in_x    <= in_x + 'd1;
end

always @(posedge clk)   // or negedge rstn)
if(sof)
    in_y    <= 'd0;
else if(recv_pixel && (in_x == xsize)) begin
    in_y    <= in_y + 'd1;
end

//--- 2: top pixel mem r/w

reg     [xy_bits-2:0]   mem_raddr, mem_waddr;
wire                    top_mem_rd  ;
wire                    top_mem_we_w;
//reg     [7:0]           last_s0_pixel;
wire    [xy_bits-2:0]   top_mem_addr_add;
wire                    rw_line_end ;
reg                     top_mem_rd_d;
reg     [15:0]          top_latch_s1;   //latch mem dout before use
reg     [7:0]           top_latch_s2;   //keep 1 pixel when there is wait cycle at recv_pixel when in_x[0]==0

assign  top_mem_ce      = top_mem_we | top_mem_rd;
assign  top_mem_we_w    = recv_pixel & in_x[0]; // pixel横坐标为奇数时，进行读
//assign  top_mem_wdata   = {s0_pixel, last_s0_pixel};
assign  top_mem_wdata   = {s0_pixel, s1_pixel}; //写的数据：上两个pixel 
assign  top_mem_addr    = (top_mem_we)? mem_waddr : mem_raddr; //地址选择
assign  top_mem_addr_add= top_mem_addr + 'd1;
assign  rw_line_end     = (top_mem_addr == xsize[xy_bits-1:1])? 1'b1 : 1'b0; //判断sram地址是否到了行尾

always @(posedge clk or negedge rstn)
if(~rstn)
    top_mem_we  <= 1'b0;
else if(top_mem_we_w)
    top_mem_we  <= 1'b1;   // pixel横坐标为奇数后一个周期拉高
else
    top_mem_we  <= 1'b0;

//always @(posedge clk)
//if(recv_pixel)
//    last_s0_pixel   <= s0_pixel;

always @(posedge clk)   // or negedge rstn)
if(sof)
    mem_waddr   <= 'd0;
else if(top_mem_we) begin
    if(rw_line_end)
        mem_waddr   <= 'd0;
    else
        mem_waddr   <= top_mem_addr_add;
end


assign  top_mem_rd  = recv_pixel & in_x[0]; //奇数读

always @(posedge clk or negedge rstn)
if(~rstn)
    top_mem_rd_d    <= 1'b0;
else
    top_mem_rd_d    <= top_mem_rd;

always @(posedge clk)   // or negedge rstn)
if(sof)
    mem_raddr   <= 'd4>>1;  //pre-fetch some pixels
else if(top_mem_rd) begin
    if(rw_line_end)
        mem_raddr   <= 'd0;
    else
        mem_raddr   <= top_mem_addr_add;
end

always @(posedge clk)   // or negedge rstn)
if(top_mem_rd_d) begin
    top_latch_s1    <= top_mem_rdata;
    top_latch_s2    <= top_latch_s1[15:8];
end


//  jpeg-ls defined neighbour locations
//  c  b  d
//  a  x
reg     [7:0]   top_c, top_b, top_d ;   //shift of top line pixel
reg     [7:0]   a_x0_last           ;
reg     [7:0]   a,b,c,d             ;
wire            y0, x0              ;
wire            x_max               ;

assign  y0      = (in_y == 'd0)? 1'b1 : 1'b0;  //第0行
assign  x0      = (in_x == 'd0)? 1'b1 : 1'b0;  //第0列
assign  x_max   = (in_x == xsize)? 1'b1 : 1'b0; //最后一列

always @(posedge clk)   // or negedge rstn)
if(recv_pixel) begin
    top_c   <= top_b; //c是之前的b
    top_b   <= top_d; //b是之前的d
end

always @(*) begin // 组合逻辑算d 的值
    if(in_x[0])   //奇数周期
        top_d   = top_latch_s1[7:0];
    else begin
        if(top_mem_rd_d)    top_d   = top_latch_s1[15:8]; //读出来结果的周期 （偶数）
        else                top_d   = top_latch_s2;       //
    end
end


always @(*) begin
    if(y0)  b = 0;
    else    b = top_b;
end

always @(*) begin
    if(y0)          d = 0;
    else if(x_max)  d = top_b;
    else            d = top_d;
end

always @(*) begin
    if(y0)          c = 0;
    else begin
        if(x0)      c = a_x0_last;
        else        c = top_c;
    end
end

always @(*) begin
    if(x0) begin
        if(y0)      a = 0;
        else        a = top_b;
    end else begin
        a = s0_pixel;
    end
end


always @(posedge clk)
if(recv_pixel && x0)
    a_x0_last   <= a;

always @(posedge clk)
if(recv_pixel) begin
    s0_a    <= a;
    s0_b    <= b;
    s0_c    <= c;
    s0_d    <= d;
    s0_pixel<= pixel;

    s0_xend <= (in_x == xsize)? 1'b1 : 1'b0;
    s0_xbeg <= (in_x == 'd0)? 1'b1 : 1'b0;
end

always @(posedge clk)   // or negedge rstn)
if(sof)
    s0_x    <= 'd0;  
else if(recv_pixel) begin
    s0_x    <= in_x;
end

always @(posedge clk)   // or negedge rstn)
if(sof)
    s0_y    <= 'd0;
else if(recv_pixel) begin
    s0_y    <= in_y;
end

always @(posedge clk or negedge rstn)
if(~rstn)
    s0_vld <= 1'b0;
else if(recv_pixel)
    s0_vld <= 1'b1;
else if(s0_ack)
    s0_vld <= 1'b0;

endmodule

