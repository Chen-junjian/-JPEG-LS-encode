//descibe: generate scan header
//         JPEG-LS pipe stage s1-s8
//         输出压缩后码流，并更新上下文模型

module jls_pipe_s1_s8 (
    // cfg info
    xsize,
    ysize,
    comp_id,

    sof,
    head_time,

    //pipeline handshake with s0
    s0_vld,
    s0_ack,
    s0_x,
    s0_y,
    s0_xbeg,
    s0_xend,
    s0_a,
    s0_b,
    s0_c,
    s0_d,

    s1_pixel,

    //pipeline handshake with syntax fifo
    s8_vld,
    s8_ack,
    s8_syntax0_vld,    // 输出数据有效
    s8_syntax0_len,    // 输出数据的有效位宽
    s8_syntax0,
    s8_syntax1_vld,
    s8_syntax1_len,
    s8_syntax1,
    s8_eof,

    //context abcn_mem r/w
    abcn_mem_rd,
    abcn_mem_raddr,
    abcn_mem_rdata,

    abcn_mem_wr,
    abcn_mem_waddr,
    abcn_mem_wdata,

    clk,
    rstn
);

parameter xy_bits = 11;

input   wire                  clk, rstn       ;
//cfg info
input   wire    [xy_bits-1:0] xsize           ;   //cnt from 0, frame x len should be 2*N pixels (odd pixel number)
input   wire    [xy_bits-1:0] ysize           ;   //cnt from 0
input   wire    [1:0]   comp_id               ;   //image color component ID  颜色的编号
input   wire            sof                   ;   //1T pulse start a frame
output  wire            head_time             ;


//pipeline handshake with s0
input   wire            s0_vld                ;   //handshake between s0 and s1
output  wire            s0_ack                ;
input   wire    [xy_bits-1:0]   s0_x          ;   //xloc within a frame, cnt from 0
input   wire    [xy_bits-1:0]   s0_y          ;   //yloc within a frame, cnt from 0
input   wire    [7:0]   s0_pixel              ;   //current pixel from JLS encoding
input   wire            s0_xbeg               ;   //1: first pixel of a line
input   wire            s0_xend               ;   //1: last pixel of a line
input   wire    [7:0]   s0_a                  ;   //neighbour_a
input   wire    [7:0]   s0_b                  ;   //neighbour_b
input   wire    [7:0]   s0_c                  ;   //neighbour_c
input   wire    [7:0]   s0_d                  ;   //neighbour_d

output  reg     [7:0]   s1_pixel              ;   //org pixel at pipe stage s1(when s1_vld=1)


//pipeline handshake with syntaxi fifo
output  reg             s8_vld          ;   //when s8_vld=1, (s8_syntax0_vld | s8_syntax1_vld) may be 1'b0
input   wire            s8_ack          ;
output  reg             s8_syntax0_vld  ;   //syntax for scan header and runcnt or run terminal
output  reg     [4:0]   s8_syntax0_len  ;   //cnt from 1, 1~16
output  reg     [15:0]  s8_syntax0      ;
output  reg             s8_syntax1_vld  ;   //syntax for golomb coding of mapped error
output  reg     [5:0]   s8_syntax1_len  ;   //cnt from 1, 1~32
output  reg     [15:0]  s8_syntax1      ;   //none stored higher bits must be 0's
output  reg             s8_eof          ;   //1: last syntax of a frame

//context abcn_mem r/w
output  wire            abcn_mem_wr     ;   //write at stage_8
output  wire    [8:0]   abcn_mem_waddr  ;
//context: a:13bit, b:7bit, c:8bit, N:6bit(cnt from 0)
output  wire    [13+7+8+6-1:0]  abcn_mem_wdata;

output  wire            abcn_mem_rd     ;   //read at stage_3
output  wire    [8:0]   abcn_mem_raddr  ;
input   wire    [13+7+8+6-1:0]  abcn_mem_rdata;



//--- 0.0: constant values
localparam  signed [8:0]    P_T1 = $signed(9'd3);
localparam  signed [8:0]    P_T2 = $signed(9'd7);
localparam  signed [8:0]    P_T3 = $signed(9'd21);

wire    [3:0]   qbpp            ;   //quant bit per pixel
wire    [4:0]   limite_normal   ;   //'d23 for 8bit image, lossless mode, used in golomb coding
wire    [8:0]   range           ;   //256 for 8bit image
wire    signed  [7:0]   min_c   ;
wire    signed  [7:0]   max_c   ;

assign  qbpp          = 'd8;      //8bit per pixel, only lossless mode
assign  limite_normal = 4*'d8 - qbpp - 'd1; // 23
assign  range         = 'd256;
assign  min_c         = 0-128;
assign  max_c         = 127;

wire [3:0] J [0:31];
assign J[ 0] = 4'd0;
assign J[ 1] = 4'd0;
assign J[ 2] = 4'd0;
assign J[ 3] = 4'd0;
assign J[ 4] = 4'd1;
assign J[ 5] = 4'd1;
assign J[ 6] = 4'd1;
assign J[ 7] = 4'd1;
assign J[ 8] = 4'd2;
assign J[ 9] = 4'd2;
assign J[10] = 4'd2;
assign J[11] = 4'd2;
assign J[12] = 4'd3;
assign J[13] = 4'd3;
assign J[14] = 4'd3;
assign J[15] = 4'd3;
assign J[16] = 4'd4;
assign J[17] = 4'd4;
assign J[18] = 4'd5;
assign J[19] = 4'd5;
assign J[20] = 4'd6;
assign J[21] = 4'd6;
assign J[22] = 4'd7;
assign J[23] = 4'd7;
assign J[24] = 4'd8;
assign J[25] = 4'd9;
assign J[26] = 4'd10;
assign J[27] = 4'd11;
assign J[28] = 4'd12;
assign J[29] = 4'd13;
assign J[30] = 4'd14;
assign J[31] = 4'd15;


//--- 0.1: generate scan header

reg     [8:0]   ini_cnt         ;
reg             ini_flag        ;
reg     [15:0]  scan_header     ;
reg             scan_header_time;

assign  head_time   = ini_flag;

always @(posedge clk or negedge rstn)
if(~rstn)
    ini_flag    <= 1'b0;
else if(sof)
    ini_flag    <= 1'b1;
else if((ini_cnt == 'd366) && s8_ack)
    ini_flag    <= 1'b0;

always @(posedge clk or negedge rstn)
if(~rstn)
    ini_cnt <= 'd0;
else if(sof)
    ini_cnt <= 'd0;
else if(ini_flag && s8_ack)
    ini_cnt <= ini_cnt + 'd1;

always @(posedge clk or negedge rstn)
if(~rstn)
    scan_header_time <= 1'b0;
else if(ini_flag && s8_ack) begin
    if(ini_cnt == 'd0)
        scan_header_time <= 1'b1;
    else if(ini_cnt == 'd5)
        scan_header_time <= 1'b0;
end

always @(*) begin
    case(ini_cnt[2:0])
    'd1:    scan_header = 16'hffda;
    'd2:    scan_header = 16'h0008;
    'd3:    scan_header = {8'h01, 6'h0, comp_id};
    'd4:    scan_header = {8'h0, 8'h0};
    'd5:    scan_header = 16'h0;
    default:scan_header = 16'h0;
    endcase
end


//--- 0.2: initial context memory
wire            abcn_mem_ini_wr     ;
wire    [8:0]   abcn_mem_ini_waddr  ;
//context: a:13bit, b:7bit, c:8bit, N:6bit(cnt from 0)
wire    [13+7+8+6-1:0]  abcn_mem_ini_wdata;

assign  abcn_mem_ini_wr     = ini_flag & s8_ack;
assign  abcn_mem_ini_waddr  = ini_cnt;
assign  abcn_mem_ini_wdata  = {13'd4, 7'h0, 8'h0, 6'd0};      //N cnt from 0

//--- 1: pipe stage_1:
//Calculate: D1/D2/D3/Q1/Q2/Q3, mode select
//For normal mode: edge-detecting predictor
//For run mode: running_flag, runend_flag, r_type, r_pred

reg             s1_vld          ;
wire            s1_ack          ;
wire            s1_halt         ;


reg     [xy_bits-1:0]   s1_x    ;
reg     [xy_bits-1:0]   s1_y    ;
reg             s1_xend         ;
reg             s1_eof          ;   //last pixel of a frame

wire    signed  [3:0]   q1_w, q2_w, q3_w        ;
reg     signed  [3:0]   s1_q1, s1_q2, s1_q3 ;   //quanted local gradient
wire            enter_run_mode  ;
wire            enc_mode_w      ;   //0:normal mode; 1:run mode
reg             s1_enc_mode     ;   //0:normal mode; 1:run mode
wire            s1_run_hit_w    ;
reg             s1_run_hit      ;   //current pixel is encoded in run mode
wire    [7:0]   normal_pred     ;   //edge-detecting predict value
wire            a_max_b_w       ;
wire    [7:0]   run_pred        ;   //run mode un-hit predict value
reg     [7:0]   s1_pred         ;   //muxed pred pixel value
wire            ritype_w        ;   //RItype in JPEG-LS spec.
reg             s1_ritype       ;   //run mode un-hit RItype
reg             s1_a_max_b      ;   //1: a > b

//----------------------------------------
// function: gradient quantization 进行量化 得到  Q [-4, 4] 的值
//----------------------------------------
function signed [3:0] func_g_quant;
input           [7:0] x1, x2;
reg     signed  [8:0] dif   ;   //-255 ~ 255
reg             sign  ;
reg             [7:0] abs   ;   //0 ~ 255

begin
    dif = {1'b0,x1} - {1'b0,x2};
    abs = (dif[8])? ((~{dif[7:0]}) + 1'b1) : dif[7:0];
    sign= dif[8];

    if(abs >= P_T3) begin
        if(sign)    func_g_quant = 4'b1100; //-4
        else        func_g_quant = 4'b0100; //4
    end else if(abs >= P_T2) begin
        if(sign)    func_g_quant = 4'b1101; //-3
        else        func_g_quant = 4'b0011; //3
    end else if(abs >= P_T1) begin
        if(sign)    func_g_quant = 4'b1110; //-2
        else        func_g_quant = 4'b0010; //2
    end else if(abs !=0) begin
        if(sign)    func_g_quant = 4'b1111; //-1
        else        func_g_quant = 4'b0001; //1
    end else begin
        func_g_quant = 4'b0000; //0
    end
end
endfunction

function [8:0]  func_pixel_pred;  //计算得到 Px
    input   [7:0]   a, b, c ;
    reg     [8:0]   c_sub_a, c_sub_b, b_sub_a;
    reg     [8:0]   abc     ;
    reg             a_max_b ;   //1: a > b; 0: a<= b

begin
    c_sub_a = {1'b0, c} - {1'b0, a};
    c_sub_b = {1'b0, c} - {1'b0, b};
    b_sub_a = {1'b0, b} - {1'b0, a};
    abc     = {1'b0, a} + {1'b0, b} - {1'b0, c};
    a_max_b = b_sub_a[8];

    func_pixel_pred[8] = a_max_b; // 1: a > b;   0: a<= b

    if((!c_sub_a[8]) && (!c_sub_b[8])) begin    //c>=a && c>=b
        if(b_sub_a[8])  //b<a
            func_pixel_pred[7:0] = b;
        else
            func_pixel_pred[7:0] = a;
    end else if(c_sub_a[8] && c_sub_b[8]) begin //c<a && c<b; result is same as JPEG_LS Spec.
        if(b_sub_a[8])  //b<a
            func_pixel_pred[7:0] = a;
        else
            func_pixel_pred[7:0] = b;
    end else begin
        func_pixel_pred[7:0] = abc[7:0];            //c within the rang of [a, b]/[b, a], must be 0~255
    end
end
endfunction


assign  s1_halt = ini_flag;
assign  s0_ack  = ((!s1_vld) | s1_ack) & (!s1_halt);

assign  q1_w            = func_g_quant(s0_d, s0_b); // Q1
assign  q2_w            = func_g_quant(s0_b, s0_c); // Q2
assign  q3_w            = func_g_quant(s0_c, s0_a); // Q3
assign  enter_run_mode  = (q1_w == 0) && (q2_w == 0) && (q3_w == 0);      // 周围a b c d都相等，具备进入游程模式的条件。
assign  enc_mode_w      = s0_xbeg? enter_run_mode : (enter_run_mode | s1_run_hit); // 进入的模式选择 ：如果是行首，当前模式完全取决于周围梯度，
                                                                                           //        如果不是行首，则如果当前像素满足进入run模式的条件，或者上一个像素已经在run模式下，则当前像素也在run模式下
assign  s1_run_hit_w    = enc_mode_w & (s0_pixel == s0_a); // 在 run model， 判断是否继续run model （结果会传给下一排，如这一拍run 且hit， 下一拍也是run，但可能不hit，再下拍退出run）

assign  {a_max_b_w, normal_pred} = func_pixel_pred(s0_a, s0_b, s0_c); //调用函数，对normal mode 进行计算预测值，得到Px

assign  ritype_w        = (s0_a == s0_b)? 1'b1 : 1'b0; // run mode 中断后的处理
assign  run_pred        = ritype_w? s0_a : s0_b; // run mode 中断后的预测值，若a=b，则预测值为a，否则为b


always @(posedge clk or negedge rstn)
if(~rstn)
    s1_vld  <= 1'b0;
else if(s0_vld && s0_ack)
    s1_vld  <= 1'b1;
else if(s1_ack)
    s1_vld  <= 1'b0;

    always @(posedge clk)   // or negedge rstn)
if(s0_vld && s0_ack) begin
    s1_x        <= s0_x;
    s1_y        <= s0_y;
    s1_pixel    <= s0_pixel;
    s1_xend     <= s0_xend;
    s1_eof      <= s0_xend && (s0_y == ysize);

    s1_q1       <= q1_w;
    s1_q2       <= q2_w;
    s1_q3       <= q3_w;

    s1_enc_mode <= enc_mode_w;  // 1： run mode; 0: normal mode
    s1_run_hit  <= s1_run_hit_w; // 有没有在 run mode 下，并且当前像素满足run hit的条件
    s1_pred     <= (enc_mode_w)? run_pred : normal_pred; // 注意 这个run_pred 是在 run mode 下，且当前像素不满足run hit的情况下，才会用到的预测值。
    s1_ritype   <= ritype_w;  // 在 run mode 但没有hit的情况下，才会用到的RItype 
    s1_a_max_b  <= a_max_b_w;
end

//当run 继续 Hit 时，s1_pred 是无效的， 当run 但没有hit时，s1_pred = run_pred 为有效预测值


//--- 2: pipe stage_2:
//Calculate: Q; runcnt, j_idx, r_limit; enc_runcnt

reg             s2_vld          ;
wire            s2_ack          ;
reg     [xy_bits-1:0]   s2_x    ;
reg     [xy_bits-1:0]   s2_y    ;
reg             s2_eof          ;
reg     [7:0]   s2_pixel        ;
reg             s2_enc_mode     ;   //0:normal mode; 1:run mode
reg             s2_run_hit      ;   //current pixel is encoded in run mode
reg     [7:0]   s2_pred         ;   //muxed pred pixel value
reg     [8:0]   s2_q_idx        ;   //0~364: normal mode context id; 365~366: run mode context id
reg             s2_q_sign       ;   //sign of Q for normal mode(0:postive; 1:negative); ritype for run mode
reg             s2_a_max_b      ;

reg     [4:0]   j_idx           ;   //index to J array
wire    [3:0]   j_val           ;
reg     [14:0]  runcnt          ;
wire    [15:0]  runcnt_add      ;
wire    [15:0]  run_rg          ;   //runcnt group size
wire            enc_runcnt      ;   //runcnt incr to runcnt group size

reg     [4:0]   s2_run_syntax_len;  //0~16
reg     [15:0]  s2_run_syntax   ;
reg     [4:0]   s2_run_glimit   ;   //7~22, after sub of "qbpp+1", used in golomb coding


function        [9:0] func_get_normal_q;
    input signed [3:0] q1, q2, q3;
    reg   signed [9:0] qs;            //[-364, 364]
    reg          s;
    reg          [8:0] q;
begin
    qs = $signed(10'd81)*q1 + $signed(10'd9)*q2 + q3;
    s = qs[9];
    q = s ? ((~qs[8:0]) + 9'd1) : qs[8:0];
    func_get_normal_q = {s, q};
end
endfunction

assign  s1_ack      = (!s2_vld) | s2_ack;

// 对于run mode 下的处理
assign  j_val       = J[j_idx];
assign  run_rg = 1 << j_val; //（2的j_val次方） 用于判断每遇到几个相同像素，就打包
assign  runcnt_add  = {1'b0, runcnt} + 'd1; // run mode 计数器
assign  enc_runcnt  = (runcnt_add == run_rg)? 1'b1 : 1'b0; // 加 1 后的计数器，是否碰到了当前的上限 $2^J 
                                                           // 如果达到了，说明正好凑齐了 $2^J$ 个相同像素！此时必须触发一次编码，向码流中输出一个“1”
                                                           // 如果没有达到，说明还没有凑齐 $2^J$ 个相同像素！此时不触发编码，继续累加计数器
always @(posedge clk or negedge rstn)
if(~rstn)
    s2_vld  <= 1'b0;
else if(s1_vld && s1_ack)
    s2_vld  <= 1'b1;
else if(s2_ack)
    s2_vld  <= 1'b0;

always @(posedge clk)   // or negedge rstn)
if(s1_vld && s1_ack) begin
    s2_x        <= s1_x         ;
    s2_y        <= s1_y         ;
    s2_eof      <= s1_eof       ;
    s2_pixel    <= s1_pixel     ;
    s2_enc_mode <= s1_enc_mode  ;
    s2_run_hit  <= s1_run_hit   ;
    s2_pred     <= s1_pred      ;
    s2_a_max_b  <= s1_a_max_b   ;
end

always @(posedge clk or negedge rstn) //计算当前像素的runcnt，若当前像素在run mode下且hit，则runcnt加1，若当前像素在run mode下但不hit，则runcnt清零
if(~rstn)
    runcnt  <= 'd0;
else if(sof || (s1_vld && s1_ack && s1_xend))
    runcnt  <= 'd0;
else if(s1_vld && s1_ack && s1_enc_mode) begin
    if(s1_run_hit) begin
        if(enc_runcnt) //达到了当前J 的上限，正好凑齐了 $2^J$ 个相同像素！触发一次编码，向码流中输出一个“1”，所以runcnt清零
            runcnt  <= 'd0;
        else
            runcnt  <= runcnt_add[14:0];
    end else begin
        runcnt <= 'd0;
    end
end


always @(posedge clk)   // or negedge rstn) // 调整 J 的大小，使压缩率达到最高
if(sof)
    j_idx   <= 'd0;
else if(s1_vld && s1_ack && s1_enc_mode) begin
    if(s1_run_hit) begin 
        if(enc_runcnt && j_idx != 'd31) // 如果触发了 enc_runcnt，且当前的 j_idx 还没达到硬件上限（31），就把 j_idx 加 1
            j_idx   <= j_idx + 'd1;
    end else begin
        if(j_idx != 0) //出现游程中断，就要把 j_idx 减 1，降低游程长度的上限，以便更快地触发编码
            j_idx   <= j_idx - 'd1;
    end
end

always @(posedge clk)   // or negedge rstn)
if(s1_vld && s1_ack) begin
    s2_run_glimit   <= 6'd32 - {1'b0, j_val} - 'd1 - qbpp - 'd1; //后面 run mode interrupt 里哥伦布编码长度限制 

    if(s1_enc_mode) begin  // run mode 的context id 只有 365/366， q_sign 表示RItype
        s2_q_idx    <= (s1_ritype)? 'd366 : 'd365;
        s2_q_sign   <= s1_ritype;
    end 
    else // normal mode 的context id 0~364， q_sign 表示Q的符号位
        {s2_q_sign, s2_q_idx}   <= func_get_normal_q(s1_q1, s1_q2, s1_q3);
end


always @(posedge clk)   // or negedge rstn)
if(s1_vld && s1_ack) begin
    if(s1_enc_mode) begin
        if(s1_run_hit) begin
            if(s1_xend) begin // 行末尾 输出编码 1
                s2_run_syntax_len   <= 'd1;
                s2_run_syntax       <= 'd1;
            end else if(enc_runcnt) begin // 到达了当前J 的上限，正好凑齐了 $2^J$ 个相同像素！触发一次编码，向码流中输出一个“1”
                s2_run_syntax_len   <= 'd1;
                s2_run_syntax       <= 'd1;
            end else begin
                s2_run_syntax_len   <= 'd0;
            end
        end else begin       // run mode 但不hit，输出编码 0 + runcnt
            s2_run_syntax_len   <= {1'b0, j_val} + 'd1; // 2~16
            s2_run_syntax       <= {1'b0, runcnt};  //1'b0, 15'hxxxx
        end
    end else begin
        s2_run_syntax_len   <= 'd0;
    end
end


//--- 3.1: pipe stage 3 ~ stage 8 signals
reg             s3_vld          ;
wire            s3_ack          ;
reg     [xy_bits-1:0]   s3_x    ;
reg     [xy_bits-1:0]   s3_y    ;
reg             s3_eof          ;
reg     [7:0]   s3_pixel        ;
reg             s3_enc_mode     ;   //0:normal mode; 1:run mode
reg             s3_run_hit      ;   //current pixel is encoded in run mode
reg     [7:0]   s3_pred         ;   //muxed pred pixel value
reg     [8:0]   s3_q_idx        ;   //0~364: normal mode context id; 365~366: run mode context id
reg             s3_q_sign       ;   //sign of Q for normal mode; ritype for run mode
reg     [4:0]   s3_run_syntax_len;  //0~16
reg     [15:0]  s3_run_syntax   ;
reg     [4:0]   s3_run_glimit   ;   //7~22, after sub of "qbpp+1", used in golomb coding
reg             s3_use_abcn_s8  ;
reg     [13+7+8+6-1:0] s3_abcn_s8;

reg             s3_a_max_b      ;
reg             s3_abcn_mem_rd_d;

reg             s4_vld          ;
wire            s4_ack          ;
reg             s4_halt         ;

reg     [xy_bits-1:0]   s4_x    ;
reg     [xy_bits-1:0]   s4_y    ;
reg             s4_eof          ;
reg     [7:0]   s4_pixel        ;
reg             s4_enc_mode     ;   //0:normal mode; 1:run mode
reg             s4_run_hit      ;   //current pixel is encoded in run mode
reg     [7:0]   s4_pred         ;   //muxed pred pixel value
reg     [8:0]   s4_q_idx        ;   //0~364: normal mode context id; 365~366: run mode context id
reg             s4_q_sign       ;   //sign of Q for normal mode; ritype for run mode
reg             s4_a_max_b      ;
reg     [13+7+8+6-1:0] s4_abcn ;   //context value for this pixel
reg     [4:0]   s4_run_syntax_len;  //0~16
reg     [15:0]  s4_run_syntax   ;
reg     [4:0]   s4_run_glimit   ;   //7~22, after sub of "qbpp+1", used in golomb coding

reg             s5_vld          ;
wire            s5_ack          ;
reg     [xy_bits-1:0]   s5_x    ;
reg     [xy_bits-1:0]   s5_y    ;
reg             s5_eof          ;
reg             s5_enc_mode     ;   //0:normal mode; 1:run mode
reg             s5_run_hit      ;   //current pixel is encoded in run mode
reg     [7:0]   s5_pred         ;   //muxed pred pixel value
reg     [8:0]   s5_q_idx        ;   //0~364: normal mode context id; 365~366: run mode context id
reg             s5_sign         ;   //SIGN flag in Spec for both normal and run mode.
reg             s5_ritype       ;   //run mode: RiType
reg     signed [8:0]   s5_err   ;   //pred error after sign correct, -255 ~255
reg     [12:0]  s5_aq_for_k_cal ;   //A[Q] or TEMP use to cal k parameter of golomb coding
reg     [12:0]  s5_a            ;   //(0~128)*63
reg     [6:0]   s5_b            ;   //normal mode: -63~0; run mode: 0~64
reg     signed [7:0]   s5_c     ;   //-128~127
reg     [6:0]   s5_n            ;   //cnt from 1; 1~64
reg     [4:0]   s5_run_syntax_len;  //0~16
reg     [15:0]  s5_run_syntax   ;
reg     [4:0]   s5_run_glimit   ;   //7~22, after sub of "qbpp+1", used in golomb coding

reg             s6_vld          ;
wire            s6_ack          ;
reg     [xy_bits-1:0]   s6_x    ;
reg     [xy_bits-1:0]   s6_y    ;
reg             s6_eof          ;
reg             s6_enc_mode     ;   //0:normal mode; 1:run mode
reg             s6_run_hit      ;   //current pixel is encoded in run mode
reg     [8:0]   s6_q_idx        ;   //0~364: normal mode context id; 365~366: run mode context id
reg             s6_sign         ;   //SIGN flag in Spec for both normal and run mode.
reg             s6_ritype       ;   //run mode: RiType
reg     [12:0]  s6_a            ;   //(0~128)*63
reg     [6:0]   s6_b            ;   //normal mode: -63~0; run mode: 0~64
reg     signed [7:0]   s6_c     ;   //-128~127
reg     [6:0]   s6_n            ;   //cnt from 1; 1~64
reg     signed [7:0] s6_err_modu;   //error after modulo, -128~127
reg     [7:0]   s6_abs_err_modu ;   //abs(error), 0~128
reg     [2:0]   s6_k            ;   //k parameter used in golomb encoding, 0~7
reg     [4:0]   s6_run_syntax_len;  //0~16
reg     [15:0]  s6_run_syntax   ;
reg     [4:0]   s6_run_glimit   ;   //7~22, after sub of "qbpp+1", used in golomb coding

reg             s7_vld          ;
wire            s7_ack          ;
reg     [xy_bits-1:0]   s7_x    ;
reg     [xy_bits-1:0]   s7_y    ;
reg             s7_eof          ;
reg             s7_enc_mode     ;   //0:normal mode; 1:run mode
reg             s7_run_hit      ;   //current pixel is encoded in run mode
reg     [8:0]   s7_q_idx        ;   //0~364: normal mode context id; 365~366: run mode context id
reg     [2:0]   s7_k            ;
wire    [8:0]   err_map_w       ;   //0~256
reg     [8:0]   s7_err_map      ;   //0~256
reg     [13+7+8+6-1:0] s7_abcn  ;
reg     [4:0]   s7_run_syntax_len;  //0~16
reg     [15:0]  s7_run_syntax   ;
reg     [4:0]   s7_run_glimit   ;   //7~22, after sub of "qbpp+1", used in golomb coding

wire    [13+7+8+6-1:0]  abcn_mem_wdata_w;



//--- 3.2: pipe stage_3:  注意进行处理数据旁路 
//Read abcn context memory
wire            s3_use_abcn_s8_w;

assign  s2_ack          = (!s3_vld) | s3_ack;
assign  s3_use_abcn_s8_w= s7_vld & (!s7_run_hit) & (s2_q_idx == s7_q_idx); //判断是否存在 读写同一个地址的冲突，如果存在，则需要进行数据旁路，使用s7的abcn值，而不是读sram的值
assign  abcn_mem_rd     = (s2_vld & s2_ack) & (!s2_run_hit) & (!s3_use_abcn_s8_w); // 没有run hit，且没有数据冒险冲突，才去读abcn_mem
assign  abcn_mem_raddr  = s2_q_idx;


always @(posedge clk or negedge rstn)
if(~rstn)
    s3_vld  <= 1'b0;
else if(s2_vld && s2_ack)
    s3_vld  <= 1'b1;
else if(s3_ack)
    s3_vld  <= 1'b0;

always @(posedge clk)   // or negedge rstn)
if(s2_vld && s2_ack) begin
    s3_x        <= s2_x         ;
    s3_y        <= s2_y         ;
    s3_eof      <= s2_eof       ;
    s3_pixel    <= s2_pixel     ;
    s3_enc_mode <= s2_enc_mode  ;
    s3_run_hit  <= s2_run_hit   ;
    s3_pred     <= s2_pred      ;
    s3_q_idx    <= s2_q_idx     ;
    s3_q_sign   <= s2_q_sign    ;
    s3_a_max_b  <= s2_a_max_b   ;

    s3_run_syntax_len<= s2_run_syntax_len;
    s3_run_syntax    <= s2_run_syntax    ;
    s3_run_glimit    <= s2_run_glimit    ;
end

always @(posedge clk)   // or negedge rstn)  
if(s2_vld && s2_ack && (!s2_run_hit)) begin
    if(s3_use_abcn_s8_w) begin              //  需要进行数据旁路，下一级s4 使用s7 写入 sram 的数据
        s3_use_abcn_s8  <= 1'b1;
        s3_abcn_s8      <= abcn_mem_wdata;
    end 
    else
        s3_use_abcn_s8  <= 1'b0;
end


always @(posedge clk)   // or negedge rstn)  // 读sram 打一拍
    s3_abcn_mem_rd_d    <= abcn_mem_rd;


//--- 4: pipe stage_4: 注意，进入s4 之后，拿着上一拍传来的s3_q_idx，和之前的s4/s5_q_idx进行比较 判断是否有数据冒险
//Get abcn context value for this pixel 有多种可能性 考虑数据冒险


assign  s3_ack  = ((!s4_vld) | s4_ack) & (!s4_halt); //拦截逻辑，只要拉高，s3_ack就会拉低，s3就会暂停
                                                     // 防止数据冒险，同一个 地址的值还没有更新，上游就不能进行访问
                                                     // 需要等待到第一个结果准备写入sram，被旁路出来输出为止

//q_idx conflit with s5/s6, halt s4 stage, until updated abcn value is ready for this q_idx.
always @(*) begin
    s4_halt = 1'b0;

    if(!s3_run_hit) begin
        if(s4_vld && (!s4_run_hit) && (s3_q_idx == s4_q_idx))
            s4_halt = 1'b1;
        else if(s5_vld && (!s5_run_hit) && (s3_q_idx == s5_q_idx))
            s4_halt = 1'b1;
        else
            s4_halt = 1'b0;
    end
end

always @(posedge clk or negedge rstn)
if(~rstn)
    s4_vld <= 1'b0;
else if(s3_vld && s3_ack)
    s4_vld <= 1'b1;
else if(s4_ack)
    s4_vld <= 1'b0;

always @(posedge clk)   // or negedge rstn)
if(s3_vld && s3_ack) begin
    s4_x        <= s3_x         ;
    s4_y        <= s3_y         ;
    s4_eof      <= s3_eof       ;
    s4_pixel    <= s3_pixel     ;
    s4_enc_mode <= s3_enc_mode  ;
    s4_run_hit  <= s3_run_hit   ;
    s4_pred     <= s3_pred      ;
    s4_q_idx    <= s3_q_idx     ;
    s4_q_sign   <= s3_q_sign    ;
    s4_a_max_b  <= s3_a_max_b   ;

    s4_run_syntax_len<= s3_run_syntax_len;
    s4_run_syntax    <= s3_run_syntax    ;
    s4_run_glimit    <= s3_run_glimit    ;
end


always @(posedge clk)   // or negedge rstn)  // 抓取最终需要使用的数据 
if(s3_vld && s3_ack) begin
    if(s6_vld && (!s6_run_hit) && (s3_q_idx == s6_q_idx))       //q_idx conflict with s7, forward path //刚刚在组合逻辑里算完，直接取
        s4_abcn <= abcn_mem_wdata_w;
    else if(s7_vld && (!s7_run_hit) && (s3_q_idx == s7_q_idx))  //q_idx conflict with s8, forward path // 计算完 准备写入sram里， 直接取
        s4_abcn <= abcn_mem_wdata;
    else if(s3_use_abcn_s8)  // 如果需要数据旁路 使用s3的abcn_s8值
        s4_abcn <= s3_abcn_s8; 
    else                     // 不用数据旁路 使用 abcn_mem_rdata
        s4_abcn <= abcn_mem_rdata;
end

//--- 5: pipe stage_5:  进行 Px 的correction 并算Error value
//Normal mode: pred correct and pred error
//Run mode:

wire            sign_run        ;   //run mode 下的 sign flag;   0: 1 in Spec.;   1:-1 in Spec. 
wire    [12:0]  s4_a            ;   //maximal: 128*63
wire    [6:0]   s4_b            ;
wire    signed [7:0]   s4_c;
wire    [6:0]   s4_n            ;

//context: a:13bit, b:7bit, c:8bit, N:6bit(cnt from 0)  上下文模型
assign  s4_n    = s4_abcn[0  +: 6] + 'd1;      // 把sram 内部的6bit[0,63]  解压缩变回7bit[1,64]
assign  s4_c    = s4_abcn[6  +: 8];            //记录历史误差的均值    
assign  s4_b    = s4_abcn[14 +: 7];
assign  s4_a    = s4_abcn[21 +: 13];

function signed [8:0]   func_errval;  //得到
input           enc_mode        ;
input           q_sign          ;   //sign of Q for normal mode; ritype for run mode
input           a_max_b         ;
input   [7:0]   pixel           ;
input   [7:0]   pred            ;
input   signed [7:0]   c        ;   //-128 ~ 127

reg     signed [9:0]   add_c;   //pred after correct, -128 ~ 383
reg     [7:0]   clip_c          ;
reg     signed [8:0]   err ;    //error before sign invert, -255~255

begin       // 计算纠正后的预测值
    if(enc_mode)  
        add_c = pred;
    else begin
        if(q_sign)  //negative
            add_c = {2'b0, pred} - {{2{c[7]}}, c}; // 算法 A.6 
        else        //positive
            add_c = {2'b0, pred} + {{2{c[7]}}, c};
    end

    //clip to [0,255] 
    if(add_c[9])        clip_c = 'd0;            //负数截断为0
    else if(add_c[8])   clip_c = 'd255;          //大于255截断为255
    else                clip_c = add_c[7:0];     // 其余的保留

    err = {1'b0, pixel} - {1'b0, clip_c};        //计算真实误差  用当前真实的像素值 pixel 减去纠正后的预测值 clip_c，得到真正的预测误差 err [-255,255]

    //error invert    再进行符号翻转
    if(enc_mode) begin
        if((!q_sign) && a_max_b)
            func_errval = (~err[8:0]) + 'b1;
        else
            func_errval = err;
    end else begin
        if(q_sign)  func_errval = (~err[8:0]) + 'b1;
        else        func_errval = err;
    end

end
endfunction


assign  s4_ack  = (!s5_vld) | s5_ack;
assign  sign_run= ((!s4_q_sign) && (s4_a_max_b))? 1'b1 : 1'b0; // run mode interrupt 下的 sign flag;   0: 1 in Spec.;   1:-1 in Spec.

always @(posedge clk or negedge rstn)
if(~rstn)
    s5_vld <= 1'b0;
else if(s4_vld && s4_ack)
    s5_vld <= 1'b1;
else if(s5_ack)
    s5_vld <= 1'b0;

always @(posedge clk)   // or negedge rstn)
if(s4_vld && s4_ack) begin
    s5_x        <= s4_x         ;
    s5_y        <= s4_y         ;
    s5_eof      <= s4_eof       ;
    s5_enc_mode <= s4_enc_mode  ;
    s5_run_hit  <= s4_run_hit   ;
    s5_pred     <= s4_pred      ;
    s5_q_idx    <= s4_q_idx     ;
    s5_sign     <= (s4_enc_mode)? sign_run : s4_q_sign; 
    s5_ritype   <= s4_q_sign    ;
    s5_err      <= func_errval(s4_enc_mode, s4_q_sign, s4_a_max_b, s4_pixel, s4_pred, s4_c);

    s5_a        <= s4_a;
    s5_b        <= s4_b;
    s5_c        <= s4_c;
    s5_n        <= s4_n;
    s5_aq_for_k_cal <= (s4_enc_mode)? ((s4_q_sign)? (s4_a + s4_n[6:1]) : s4_a) : s4_a; //这里要区分 run / normal 的a[Q]值

    s5_run_syntax_len<= s4_run_syntax_len;
    s5_run_syntax    <= s4_run_syntax    ;
    s5_run_glimit    <= s4_run_glimit    ;
end

//--- 6: pipe stage_6:
//Calculate: perd error modulo, k parameter in golomb coding

wire    signed [7:0] err_modu_w ;   //error after modulo, -128~127 // 对error 进行modulation

function signed [7:0] func_err_modu;
input   signed [8:0]   err ;    //-255 ~ 255

reg     [7:0]   s1          ;   //0~255

begin
    //step 1
    /*
    if(err[8])  s1 = err + range;
    else        s1 = err;
    */
    s1 = err[7:0];      //simplified for lossless mode

    //step 2
    if(s1[7]) begin //s1 >= 128, change to neg number
        //func_err_modu = {1'b0, s1} - range;
        func_err_modu = s1[7:0];    //simplified for lossless mode
    end else begin  //s1: [0,127], no change
        func_err_modu = s1[7:0];
    end
end
endfunction

function [2:0]  func_cal_k;  //计算得到  k 是满足 N * 2^k >= A 的最小整数   run/ normal 一致的方法
input   [12:0]  aq  ;   //A[q] in the range of [0*(N[q]-1), 128*(N[q]-1)]
input   [6:0]   nq  ;   //N[q] in range of [0,64]
begin
    if(nq >= aq)                    func_cal_k = 0;
    else if({nq, 1'b0} >= aq)       func_cal_k = 1;
    else if({nq, 2'b0} >= aq)       func_cal_k = 2;
    else if({nq, 3'b0} >= aq)       func_cal_k = 3;
    else if({nq, 4'b0} >= aq)       func_cal_k = 4;
    else if({nq, 5'b0} >= aq)       func_cal_k = 5;
    else if({nq, 6'b0} >= aq)       func_cal_k = 6;
    else                            func_cal_k = 7;
    /*
    else if({nq, 7'b0} >= aq)       func_cal_k = 7;
    else if({nq, 8'b0} >= aq)       func_cal_k = 8;
    else if({nq, 9'b0} >= aq)       func_cal_k = 9;
    else if({nq, 10'b0} >= aq)      func_cal_k = 10;
    else if({nq, 11'b0} >= aq)      func_cal_k = 11;
    else if({nq, 12'b0} >= aq)      func_cal_k = 12;
    else if({nq, 13'b0} >= aq)      func_cal_k = 13;
    else                            func_cal_k = 14;
    */
end
endfunction


assign  s5_ack          = (!s6_vld) | s6_ack;
assign  err_modu_w      = func_err_modu(s5_err);

always @(posedge clk or negedge rstn)
if(~rstn)
    s6_vld <= 1'b0;
else if(s5_vld && s5_ack)
    s6_vld <= 1'b1;
else if(s6_ack)
    s6_vld <= 1'b0;

always @(posedge clk)   // or negedge rstn)
if(s5_vld && s5_ack) begin
    s6_x        <= s5_x         ;
    s6_y        <= s5_y         ;
    s6_eof      <= s5_eof       ;
    s6_enc_mode <= s5_enc_mode  ;
    s6_run_hit  <= s5_run_hit   ;
    s6_q_idx    <= s5_q_idx     ;
    s6_sign     <= s5_sign      ;
    s6_ritype   <= s5_ritype    ;
    s6_a        <= s5_a         ;
    s6_b        <= s5_b         ;
    s6_c        <= s5_c         ;
    s6_n        <= s5_n         ;
    s6_err_modu <= err_modu_w   ;
    s6_abs_err_modu<= (err_modu_w[7])? ({1'b0, ~err_modu_w[6:0]} + 1'b1) : {1'b0, err_modu_w[6:0]}; //得到 err_val modulation 的绝对值
    s6_k        <= func_cal_k(s5_aq_for_k_cal, s5_n);

    s6_run_syntax_len<= s5_run_syntax_len;
    s6_run_syntax    <= s5_run_syntax    ;
    s6_run_glimit    <= s5_run_glimit    ;
end


//--- 7: pipe stage_7:
//Calculate: mapped pred error, abcn context update for this pixel;

function [8:0]  func_err_map; // 进行误差映射 将$[-128, 127] 映射到正数 得到算法里的 MErrval  run / normal 是不一样的
input           enc_mode    ;
input           ritype      ;
input   [2:0]   k           ;   //0~7
input   [6:0]   b           ;   //normal mode: -63~0; run mode: 0~64
input   [6:0]   n           ;   //1~64

input   signed  [7:0]   err ;   //modulo error, -128~127
input   [7:0]   abs_err     ;   //0~128

reg     map                 ;
reg     [7:0]   bx2         ;
reg     [6:0]   neg_n       ;
reg     [8:0]   neg_n_sub_bx2   ;   //normal mode cal
reg     [8:0]   bx2_add_neg_n   ;   //run mode, -64~64
reg             err_big_0       ;

begin
    bx2         = {b, 1'b0};
    neg_n       = ~n + 1'b1;  // -n 
    neg_n_sub_bx2 = {{2{neg_n[6]}}, neg_n} - {bx2[7], bx2}; //选用-n -2b >=0 ,比起 n+2b <= 0 也是有优化
    bx2_add_neg_n = {1'b0, bx2} + {{2{neg_n[6]}}, neg_n};
    err_big_0   = (!err[7]) & (err[6:0] != 'd0);

    if(enc_mode) begin  //run mode
        if((err != 0) && (err_big_0 == ((k == 0) && bx2_add_neg_n[8])))
            map = 1;
        else
            map = 0;
            
        func_err_map = {abs_err, 1'b0} - ritype - map;  //0~256
    end else begin      //normal mode
        if((k==0) && (!neg_n_sub_bx2[8]))
            map = 1;
        else
            map = 0;
            
        if(err[7])  //err < 0
            func_err_map = {abs_err, 1'b0} - map -1;    //0~255
        else
            func_err_map = {abs_err, 1'b0} + map;       //0~255
    end
end
endfunction


function [13+7+8+6-1:0] func_abcn_update;
input           enc_mode    ;
input   signed  [7:0] err   ;   //modulo error, -128~127
input   [7:0]   abs_err     ;   //0~128
input   [12:0]  a_in        ;   //(0~128)*63
input   [6:0]   b_in        ;   //normal mode: -63~0; run mode: 0~64
input   signed  [7:0]   c_in    ;   //-128~127
input           [6:0]   n_in    ;   //cnt from 1; 1~64
input           [8:0]   err_map ;   //0~256
input                   ritype  ;

//normal mode signals
reg     [13:0]  a_add           ;
reg     signed  [8:0]   b_add   ;   //-191 ~ 127
reg     [12:0]  a_reset         ;
reg     signed  [8:0]   b_reset ;   //-191 ~ 127
reg     [6:0]   n_reset         ;
reg     signed  [8:0]   b_add_n_reset;  //-190 ~ 191
reg     signed  [8:0]   b_add_2n_reset; //-189 ~ 255
reg     signed  [8:0]   b_sub_n_reset;  //-255 ~ 127

reg     [12:0]  a               ;
reg     [6:0]   b               ;
reg     [7:0]   c               ;
reg     [5:0]   n               ;   //change to cnt from 0 从[1,64] 改为映射为 [0-63],节省1bit
reg     [8:0]   a_add_run       ;

begin
    // 先对a 进行处理 分为run mode 和 normal mode
    a_add_run   = err_map + 'd1 - ritype;

    if(enc_mode)
        a_add = {1'b0, a_in} + a_add_run[8:1];
    else
        a_add = {1'b0, a_in} + abs_err;

    // 对b 进行处理 
    if(enc_mode) begin   // 有疑问 在进行run mode 时 对B [Q] 没有操作呀 这部分是做什么？
        if(err[7])  b_add = b_in + 'd1;
        else        b_add = b_in;
    end 
        else begin
        b_add = {{2{b_in[6]}}, b_in} + {err[7], err}; // b[Q]= b[Q]+error
    end

    if(n_in[6]) begin // 代表 N[Q] = 64
        n_reset = 'd33;
        b_reset = {b_add[8], b_add[8:1]};
        a_reset = a_add[13:1];
    end else begin
        n_reset = n_in + 'd1;
        b_reset = b_add;
        a_reset = a_add[12:0];
    end //做完 算法里的 A.12

    b_add_n_reset = b_reset + {2'b0, n_reset};
    b_add_2n_reset= b_reset + {1'b0, n_reset, 1'b0};
    b_sub_n_reset = b_reset - {2'b0, n_reset};

    //update of a
    a = a_reset;

    //update of b
    if(!enc_mode) begin
        if(b_add_2n_reset <= 0)
            b = ~n_reset + 'd2;     // b 在加上 n后，仍小于-n， b = -n +1： rtl上 = ~n+1+1
        else if(b_add_n_reset <=0) 
            b = b_add_n_reset[6:0]; // b 在加上 n后，大于 -n， b=  b + n
        else if(b_sub_n_reset > 0)
            b = 'd0;                // b 一开始大于0，在b-n之后还大于0， b=0
        else if(b_reset > 0)
            b = b_sub_n_reset[6:0]; // b 一开始大于0，在b-n之后小于0， b = b-n
        else
            b = b_reset[6:0];       // 无需修正 ：既不满足 B <= -N，也不满足 B > 0, 即 B 在 （-N,0]
    end else begin
        b = b_reset[6:0];           // run mode : b 不变
    end 

    //update of c c的更新，对应 A.13
    if((b_add_n_reset <= 0) && (c_in != min_c))
        c = c_in - 'd1;
    else if((b_reset > 0) && (c_in != max_c))
        c = c_in + 'd1;
    else
        c= c_in;

    //update of N //把[1,64]映射到[0,63]
    n = n_reset - 'd1;  //change to cnt from 0

    func_abcn_update = {a, b, c, n};
end
endfunction


assign  s6_ack = (!s7_vld) | s7_ack;
assign  err_map_w       = func_err_map(s6_enc_mode, s6_ritype, s6_k, s6_b,
                                       s6_n, s6_err_modu, s6_abs_err_modu); //进行 error mapping 之后的值

assign  abcn_mem_wdata_w= func_abcn_update(s6_enc_mode, s6_err_modu, s6_abs_err_modu, s6_a,
                                           s6_b, s6_c, s6_n, err_map_w, s6_ritype); //准备写入sram

always @(posedge clk or negedge rstn)
if(~rstn)
    s7_vld  <= 1'b0;
else if(s6_vld && s6_ack)
    s7_vld  <= 1'b1;
else if(s7_ack)
    s7_vld  <= 1'b0;

always @(posedge clk)   // or negedge rstn)
if(s6_vld && s6_ack) begin
    s7_x        <= s6_x         ;
    s7_y        <= s6_y         ;
    s7_eof      <= s6_eof       ;
    s7_enc_mode <= s6_enc_mode  ;
    s7_run_hit  <= s6_run_hit   ;
    s7_q_idx    <= s6_q_idx     ;
    s7_k        <= s6_k         ;
    s7_err_map  <= err_map_w    ;
    s7_abcn     <= abcn_mem_wdata_w;

    s7_run_syntax_len<= s6_run_syntax_len;
    s7_run_syntax    <= s6_run_syntax    ;
    s7_run_glimit    <= s6_run_glimit    ;
end


//--- 8: pipe stage s8 ：最终算出压缩码流

reg     [xy_bits-1:0]   s8_x    ;
reg     [xy_bits-1:0]   s8_y    ;
reg             s8_enc_mode     ;   //0:normal mode; 1:run mode
reg             s8_run_hit      ;   //current pixel is encoded in run mode

wire            s8_abcn_mem_wr  ;   // 只有Normal Mode 或游程中断时才写回

wire    [4:0]   limit           ;   //7~23
reg     [5:0]   golomb_len      ;   //1~32, cnt from 1
reg     [15:0]  golomb_val      ;   //just store lower bits, as higher bits in val are all 0s
reg     [8:0]   err_map_sf      ;
wire    [7:0]   one_lf_k        ;   //1 << s7_k
wire    [6:0]   mask_flag       ;
wire    [6:0]   err_map_mask    ;   //get kbit (k within [0,7]) low bits of err_map
wire    [7:0]   err_map_sub1    ;   //0~255

assign  s7_ack          = (!s8_vld) | s8_ack;
assign  s8_abcn_mem_wr  = s7_vld & s7_ack & (!s7_run_hit); // run hit 模式不写入

assign  abcn_mem_wr     = abcn_mem_ini_wr | s8_abcn_mem_wr;
assign  abcn_mem_waddr  = (abcn_mem_ini_wr)? abcn_mem_ini_waddr : s7_q_idx;
assign  abcn_mem_wdata  = (abcn_mem_ini_wr)? abcn_mem_ini_wdata : s7_abcn;

always @(posedge clk or negedge rstn)
if(~rstn)
    s8_vld  <= 1'b0;
else if(scan_header_time)   //not in pipeline, send out scan header
    s8_vld  <= 1'b1;
else if(s7_vld && s7_ack)
    s8_vld  <= 1'b1;
else if(s8_ack)
    s8_vld  <= 1'b0;

// 做 Golomb-Rice 编码， 作为normal 模式的 码流  算法 A.5.3
assign  one_lf_k    = 1 << s7_k;    // 除数
assign  limit       = (s7_enc_mode)? s7_run_glimit : limite_normal;  //规定界限
assign  mask_flag   = one_lf_k - 'd1; //
assign  err_map_mask= s7_err_map[6:0] & mask_flag;  // 实际上，为除法后的余数
assign  err_map_sub1= s7_err_map - 'd1;             // err_map -1 

always @(*) begin
    err_map_sf  = s7_err_map >> s7_k;   // 把err_map 除以 2 的k 次方
end

// 哥伦布编码的算法是一样的 只是run / normal 的 error_map , limit 都不一样。 （run 的limit 在s2 就准备好）

always @(*) begin                       // 得到对应的哥伦布长度和哥布林值
    if(err_map_sf < limit) begin             // 没有超过固定界限 （23）
        golomb_len = err_map_sf[4:0] + 'd1 + s7_k;          //limit in the range of [7,23]
        golomb_val = one_lf_k | err_map_mask;               //{{err_map_sf[4:0]{1'b0}}, 1'b1, 0~7'hxx} 一样的： 前面的0 自动补齐，这里的值是那个1'b1 + k’b MError
    end else begin                           // 超过固定界限
        golomb_len = {1'b0, limit} + 'd1 + qbpp;            // 32 bit
        golomb_val = ('d1 << qbpp) | (err_map_sub1);        //{{limit{1'b0}}, 1'b1, 8'hxx} 高位bit 补0，这里一共 9bit 9'b1_err_map_sub1的后8bit
    end
end

always @(posedge clk)   // or negedge rstn)
if(s7_vld && s7_ack) begin
    s8_x        <= s7_x         ;
    s8_y        <= s7_y         ;
    s8_eof      <= s7_eof       ;
    s8_enc_mode <= s7_enc_mode  ;
    s8_run_hit  <= s7_run_hit   ;
end

//--- syntax for scan header and runcnt or run terminal
always @(posedge clk or negedge rstn)
if(~rstn)
    s8_syntax0_vld  <= 1'b0;
else if(scan_header_time)
    s8_syntax0_vld  <= 1'b1;
else if(s7_vld && s7_ack && (s7_run_syntax_len != 'd0)) begin
    s8_syntax0_vld  <= 1'b1;
end else if(s8_ack)
    s8_syntax0_vld  <= 1'b0;

always @(posedge clk)   // or negedge rstn) // 这条通道是对scan header，run mode 的编码
if(scan_header_time) begin
    s8_syntax0_len  <= 'd16;
    s8_syntax0      <= scan_header;
end else if(s7_vld && s7_ack) begin
    s8_syntax0_len  <= s7_run_syntax_len;
    s8_syntax0      <= s7_run_syntax;
end

//--- syntax for golomb coding of mapped error 这条通道，是对normal mode 以及run mode 的中断输出的
always @(posedge clk or negedge rstn)
if(~rstn)
    s8_syntax1_vld <= 1'b0;
else if(s7_vld && s7_ack && (!s7_run_hit))
    s8_syntax1_vld <= 1'b1;
else if(s8_ack)
    s8_syntax1_vld <= 1'b0;

always @(posedge clk)   // or negedge rstn)
if(s7_vld && s7_ack && (!s7_run_hit)) begin
    s8_syntax1_len <= golomb_len;
    s8_syntax1     <= golomb_val;
end

endmodule
