//----------------------------------------------------------------------------//
// Description  : 
// 1: JPEG_LS encoder, support 8bit depth, lossless encode only;
// 2: SW generate image header, hw generate scan header;
// 3: Each trigger of HW will encode a color component of a image;
//----------------------------------------------------------------------------//

module jls_enc(
    //cfg info
    xsize           ,
    ysize           ,

    //image pixel recv
    sof             ,
    comp_id         ,
    pixel_vld       ,
    pixel           ,
    pixel_ack       ,

    //handshake with stream output
    stream_vld      ,
    stream_bytes    ,
    stream_eof      ,
    stream_ack      ,
    stream_len      ,
    stream          ,

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
input   wire    [1:0]   comp_id         ;   //color component id of this sof trigger
input   wire            pixel_vld       ;
input   wire    [7:0]   pixel           ;
output  wire            pixel_ack       ;

//handshake with stream output
output  wire            stream_vld      ;   //1 or 2 byte is valid in stream
output  wire            stream_bytes    ;   //0: only stream[7:0] is valid; 1: stream[15:0] is valid
output  wire            stream_eof      ;   //high level active: stream output end for a frame; align with last stream_vld or later
input   wire            stream_ack      ;
output  wire    [23:0]  stream_len      ;   //stream byte length, cnt from 1
output  wire    [15:0]  stream          ;   //first bit on bit[7] of a byte, little-endian byte order


//top line mem r/w
wire            top_mem_ce      ;   //mem ce
wire            top_mem_we      ;
wire    [xy_bits-2:0]top_mem_addr;  //each addr store 2 pixels
wire    [15:0]  top_mem_wdata   ;
wire    [15:0]  top_mem_rdata   ;

//pipeline handshake with downstream
wire            head_time       ;
wire            s0_vld          ;   //handshake between s0 and s1
wire            s0_ack          ;
wire    [xy_bits-1:0]   s0_x    ;   //xloc within a frame, cnt from 0
wire    [xy_bits-1:0]   s0_y    ;   //yloc within a frame, cnt from 0
wire    [7:0]   s0_pixel        ;   //current pixel from JLS encoding
wire            s0_xend         ;   //1: last pixel of a line
wire            s0_xbeg         ;   //1: first pixel of a line
wire    [7:0]   s0_a            ;   //neighbour_a
wire    [7:0]   s0_b            ;   //neighbour_b
wire    [7:0]   s0_c            ;   //neighbour_c
wire    [7:0]   s0_d            ;   //neighbour_d
wire    [7:0]   s1_pixel        ;   //org pixel at pipe stage s1(when s1_vld=1)

//pipeline handshake with syntaxi fifo
wire            s8_vld          ;
wire            s8_ack          ;
wire            s8_syntax0_vld  ;   //syntax for scan header and runcnt or run terminal
wire    [4:0]   s8_syntax0_len  ;   //cnt from 1, 1~16
wire    [15:0]  s8_syntax0      ;
wire            s8_syntax1_vld  ;   //syntax for golomb coding of mapped error
wire    [5:0]   s8_syntax1_len  ;   //cnt from 1, 1~32
wire    [15:0]  s8_syntax1      ;
wire            s8_eof          ;

//context abcn_mem r/w
wire            abcn_mem_wr     ;   //write at stage_8
wire    [8:0]   abcn_mem_waddr  ;
//context: a:13bit, b:7bit, c:8bit, N:6bit(cnt from 0)
wire    [13+7+8+6-1:0]  abcn_mem_wdata;
wire            abcn_mem_rd     ;   //read at stage_3
wire    [8:0]   abcn_mem_raddr  ;
wire    [13+7+8+6-1:0]  abcn_mem_rdata;


jls_recv_pixel #(.xy_bits(xy_bits)) u_jls_recv_pixel(
    //cfg info
    .xsize          (xsize          ),
    .ysize          (ysize          ),

    //image pixel recv
    .sof            (sof            ),
    .pixel_vld      (pixel_vld      ),
    .pixel          (pixel          ),
    .pixel_ack      (pixel_ack      ),

    //top line mem r/w
    .top_mem_ce     (top_mem_ce     ),
    .top_mem_we     (top_mem_we     ),
    .top_mem_addr   (top_mem_addr   ),
    .top_mem_wdata  (top_mem_wdata  ),
    .top_mem_rdata  (top_mem_rdata  ),

    //pipeline handshake with downstream
    .s0_vld         (s0_vld         ),
    .s0_ack         (s0_ack         ),
    .s0_x           (s0_x           ),
    .s0_y           (s0_y           ),
    .s0_pixel       (s0_pixel       ),
    .s0_xend        (s0_xend        ),
    .s0_xbeg        (s0_xbeg        ),
    .s0_a           (s0_a           ),
    .s0_b           (s0_b           ),
    .s0_c           (s0_c           ),
    .s0_d           (s0_d           ),
                                    
    .s1_pixel       (s1_pixel       ),
                                    
    .clk            (clk            ),
    .rstn           (rstn           )
);

jls_pipe_s1_s8 #(.xy_bits(xy_bits)) u_jls_pipe_s1_s8(
    //cfg info
    .xsize          (xsize          ),
    .ysize          (ysize          ),
    .comp_id        (comp_id        ),
                                    
    .sof            (sof            ),
    .head_time      (head_time      ),
    
    //pipeline handshake with s0
    .s0_vld         (s0_vld         ),
    .s0_ack         (s0_ack         ),
    .s0_x           (s0_x           ),
    .s0_y           (s0_y           ),
    .s0_pixel       (s0_pixel       ),
    .s0_xbeg        (s0_xbeg        ),
    .s0_xend        (s0_xend        ),
    .s0_a           (s0_a           ),
    .s0_b           (s0_b           ),
    .s0_c           (s0_c           ),
    .s0_d           (s0_d           ),
    .s1_pixel       (s1_pixel       ),

    //pipeline handshake with syntax fifo
    .s8_vld         (s8_vld         ),
    .s8_ack         (s8_ack         ),
    .s8_syntax0_vld (s8_syntax0_vld ),
    .s8_syntax0_len (s8_syntax0_len ),
    .s8_syntax0     (s8_syntax0     ),
    .s8_syntax1_vld (s8_syntax1_vld ),
    .s8_syntax1_len (s8_syntax1_len ),
    .s8_syntax1     (s8_syntax1     ),
    .s8_eof         (s8_eof         ),

    //context abcn_mem r/w
    .abcn_mem_wr    (abcn_mem_wr    ),
    .abcn_mem_waddr (abcn_mem_waddr ),
    .abcn_mem_wdata (abcn_mem_wdata ),
    .abcn_mem_rd    (abcn_mem_rd    ),
    .abcn_mem_raddr (abcn_mem_raddr ),
    .abcn_mem_rdata (abcn_mem_rdata ),
                                    
    .clk            (clk            ),
    .rstn           (rstn           ) 
);

jls_syntax_to_stream u_jls_syntax_to_stream(
    .sof            (sof            ),
    .head_time      (head_time      ),

    //pipeline handshake with syntax fifo
    .s8_vld         (s8_vld         ),
    .s8_ack         (s8_ack         ),
    .s8_syntax0_vld (s8_syntax0_vld ),
    .s8_syntax0_len (s8_syntax0_len ),
    .s8_syntax0     (s8_syntax0     ),
    .s8_syntax1_vld (s8_syntax1_vld ),
    .s8_syntax1_len (s8_syntax1_len ),
    .s8_syntax1     (s8_syntax1     ),
    .s8_eof         (s8_eof         ),

    //handshake with stream output
    .stream_vld     (stream_vld     ),
    .stream_bytes   (stream_bytes   ),
    .stream_eof     (stream_eof     ),
    .stream_ack     (stream_ack     ),
    .stream_len     (stream_len     ),
    .stream         (stream         ),
                                    
    .clk            (clk            ),
    .rstn           (rstn           ) 
);



//real usage
//top pixel line buffer
spram_generic #(.ADDR_BITS(xy_bits-1), .ADDR_AMOUNT(1 << (xy_bits-1)), .DATA_BITS(16)) u_top_buf(
    .clk            (clk            ),
    .en             (top_mem_ce     ),
    .we             (top_mem_we     ),
    .addr           (top_mem_addr   ),
    .din            (top_mem_wdata  ),
    
    .dout           (top_mem_rdata  )
);   

//abcn context memory(367 depth is ok)
dpram_generic #(.ADDR_BITS(9), .ADDR_AMOUNT(368), .DATA_BITS(13+7+8+6)) u_abcn_context(
    .clka           (clk            ),
    .ena            (abcn_mem_wr    ),
    .wea            (abcn_mem_wr    ),
    .dina           (abcn_mem_wdata ),
    .addra          (abcn_mem_waddr ),
    .clkb           (clk            ),
    .enb            (abcn_mem_rd    ),
    .addrb          (abcn_mem_raddr ),
    .doutb          (abcn_mem_rdata )
); 

endmodule

