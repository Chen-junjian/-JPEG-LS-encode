module jls_syntax_to_stream(
    sof             ,
    head_time       ,

    //pipeline handshake with syntax fifo
    s8_vld          ,
    s8_ack          ,
    s8_syntax0_vld  ,
    s8_syntax0_len  ,
    s8_syntax0      ,
    s8_syntax1_vld  ,
    s8_syntax1_len  ,
    s8_syntax1      ,
    s8_eof          ,

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


input   wire            clk, rstn       ;
input   wire            sof             ;
input   wire            head_time       ;   //send scan header time, no need insert 1'b0 after 8'hff

//pipeline handshake with syntaxi fifo
input   wire            s8_vld          ;   //when s8_vld=1, (s8_syntax0_vld | s8_syntax1_vld) may be 1'b0
output  reg             s8_ack          ;
input   wire            s8_syntax0_vld  ;   //syntax for scan header and runcnt or run terminal
input   wire    [4:0]   s8_syntax0_len  ;   //cnt from 1, 1~16

//header: 16'hxxxx;
//run encode: 1'b0, 15~1'hxxxx
input   wire    [15:0]  s8_syntax0      ;   //invalid higher bits must be 0's

input   wire            s8_syntax1_vld  ;   //syntax for golomb coding of mapped error
input   wire    [5:0]   s8_syntax1_len  ;   //cnt from 1, 1~32
//golomb encode: 1'b1, 8~0'hxx
input   wire    [15:0]  s8_syntax1      ;   //invalid higher bits must be 0's
input   wire            s8_eof          ;   //1: last syntax of a frame

//handshake with stream output
output  reg             stream_vld      ;   //1 or 2 byte is valid in stream //每次输出1/2 byte
output  reg             stream_bytes    ;   //0: only stream[7:0] is valid; 1: stream[15:0] is also valid
output  reg             stream_eof      ;   //high level active: stream output end for a frame; align with stream_vld or later
input   wire            stream_ack      ;
output  wire    [23:0]  stream_len      ;   //stream byte length, cnt from 1; valid before last stream_vld. signal has bug on it,
                                            //missing the counting of 1'b0.
output  reg     [15:0]  stream          ;   //first bit on bit[7] of a byte, little-endian byte order

//--- 1: receive syntax and store to fifo
reg             sta             ;
parameter       s_s0 = 1'b0,    //push first syntax
                s_s1 = 1'b1;    //push sencond syntax

wire            fifo_ful        ;
reg     [5:0]   mux_syntax_len  ;
reg     [15:0]  mux_syntax_val  ;
wire            fifo_push       ;
wire    [16+6:0]    fifo_wdata  ;
wire            fifo_pop        ;
wire    [16+6:0]    fifo_rdata  ;   //valid same cycle as fifo_pop
wire            fifo_empty      ;
reg             syntax_eof      ;   //1T delay of first fifo push of this s8_vld
wire            sf_reg_flush    ;   //flush bits in sf_reg when frame end
reg     [24+3-1:0]  bits_acc    ;

assign  sf_reg_flush= syntax_eof & (sta == s_s0); // 一帧结束后 写入特定的码流
//                                  flush bit in sf_reg : normal syntax shift in
assign  fifo_wdata  = (sf_reg_flush)? {1'b1, 6'h7, 16'h0} : {1'b0, mux_syntax_len, mux_syntax_val};
assign  fifo_push   = (!fifo_ful) & (   (s8_vld & (((sta == s_s0) & (s8_syntax0_vld | s8_syntax1_vld)) | (sta == s_s1)))
                                      | (syntax_eof & (sta == s_s0)) ); // FIFO 在非满的时候，满足在s1 状态，或s0状态同时两组通道有数据或一帧结束时 进行写

// 统计这一帧总共有多少bit bug ：没有统计1‘b0 的bit
always @(posedge clk)   // or negedge rstn)
if(sof)
    bits_acc    <= 'd0;
else if(s8_vld && s8_ack) 
    bits_acc    <= bits_acc + ((s8_syntax0_vld)? s8_syntax0_len : 'd0) + ((s8_syntax1_vld)? s8_syntax1_len : 'd0);
else if(sf_reg_flush)
    bits_acc    <= bits_acc + 'd7;  //align to byte length  注意看stream_len 的赋值，+7 的作用是用来进位


//判断一帧是否结束
always @(posedge clk or negedge rstn)
if(~rstn)
    syntax_eof  <= 1'b0;
else if(s8_vld && s8_ack && s8_eof)
    syntax_eof  <= 1'b1;
else if((!fifo_ful) && (sta == s_s0))
    syntax_eof  <= 1'b0;

// 状态机维护，若同时两个通道有数据，进行先后写入
always @(posedge clk or negedge rstn)
if(~rstn)
    sta     <= s_s0;
else if(sof)
    sta     <= s_s0;
else if(s8_vld && (!fifo_ful)) begin
    case(sta)
        s_s0:   begin
            if(s8_syntax0_vld && s8_syntax1_vld)
                sta     <= s_s1;
            else
                sta     <= s_s0;
        end

        s_s1:   begin
            sta     <= s_s0;
        end
    endcase
end

always @(*) begin
    s8_ack = 1'b0;

    if(!fifo_ful) begin
        if(sta == s_s1)
            s8_ack = 1'b1;
        else begin
            if(s8_syntax0_vld && s8_syntax1_vld)
                s8_ack = 1'b0;
            else
                s8_ack = 1'b1;
        end
    end
end

// 在s1 的时候写1，其余时候优先写0
always @(*) begin
    if(sta == s_s1) begin
        mux_syntax_val = s8_syntax1;
        mux_syntax_len = s8_syntax1_len;
    end else begin
        if(s8_syntax0_vld) begin
            mux_syntax_val = s8_syntax0;
            mux_syntax_len = s8_syntax0_len;
        end else begin
            mux_syntax_val = s8_syntax1;
            mux_syntax_len = s8_syntax1_len;
        end
    end
end

sync_fifo #(.FIFO_WIDTH(1+6+16), .FIFO_DEPTH(4), .FIFO_ADDR_BIT(2)) u_syntax_fifo(
    .fifo_wr        (fifo_push  ),
    .fifo_rd        (fifo_pop   ),
    .fifo_din       (fifo_wdata ),
    .fifo_do        (fifo_rdata ),
    .fifo_ful       (fifo_ful   ),
    .fifo_empty     (fifo_empty ),
    .clk            (clk        ),
    .rstn           (rstn       )
);

//--- 2: pop out and combine to 16bit stream  使用寄存器 将fifo 读出来的数据进行寄存
//note: maximal shift in 16bit a cycle; if fifo_len > 16bit, shift in
//(fifo_len - 16) bits in first cycle (shift in val must be 0's), then shift in 16bits in second cycle;

//--- 2.1 stage_0: shift in
reg             s0_vld          ;   //once some bits shift in sf_reg or 16bits is valid in sf_reg, s0_vld will active
wire            s0_ack          ;

wire    [5:0]   fifo_len        ;   //1~32
wire    [15:0]  fifo_val        ;
wire            fifo_flush      ;
wire            sf_bits         ;
reg     [4:0]   sf_len          ;   //1~16
reg     [15:0]  sf_val          ;   //invalid higher bits must be 0's
wire    [4:0]   remain_len      ;   //constant 16, remain syntax length not shift in
wire            big_16bit       ;

reg     [4:0]   bits_cnt        ;   //bits number remain in sf_reg, 0~24 表示寄存器内部有多少个有效数据
reg     [23:0]  sf_reg          ;
wire    [22:0]  nxt_sf_reg      ;   //after shift in bits  拼接syntax 的组合逻辑
reg     [23:0]  nxt_sf_reg_add0 ; //after add 1'b0 after 8'hff byte 按照要求补0 的组合逻辑 补完0后，update sf_reg


wire    [4:0]   nxt_bits_cnt    ;   //0~23
wire    [4:0]   nxt_bits_cnt_add0;  //0~24
reg             add0_flag       ;
reg             sf_eof          ;   //align with s0_vld


reg     [1:0]   sf_sta          ;
reg             back_flag       ;   //0: back to s_sf_h after s_out_2; 1: back to s_sf_l after s_out_2
parameter [1:0] s_sf_h = 'd0,       //shift in higher bits with length>16: ((fifo_len>16)? (fifo_len - 16) : fifo_len)
                s_sf_l = 'd1,       //shift in lower 16bits if fifo_len>16 
                s_out_2= 'd2;       //output second 8bits 当前sf_reg 到达24bit， 不从fifo，接数据 先从sf_reg输出16bit数据，下个周期把剩下8bit输出

// 在fifo 读出的时候 组合逻辑得到
assign  fifo_val        = fifo_rdata[0 +: 16];  //读出的数据
assign  fifo_len        = fifo_rdata[16 +: 6];  //读出有效的长度
assign  fifo_flush      = fifo_rdata[22];       // 判断是否为最后
assign  big_16bit       = fifo_len[5] | (fifo_len[4] & (fifo_len[3:0] != 'd0)); // 长度为16-32
assign  fifo_pop        = (!fifo_empty) & ( ((sf_sta == s_sf_h) & (!big_16bit)) | (sf_sta == s_sf_l) ) & 
                          ((!s0_vld) | s0_ack); //推出数据 ：fifo 不空 & 下游通畅 & 如果小于16bit，一拍吃完，如果大于16，要第二拍才吃完
assign  sf_bits         = ((!fifo_empty) & (sf_sta == s_sf_h)) | (sf_sta == s_sf_l); //告诉sf_reg 当前有没有数据要移入


assign  remain_len = 5'd16;
assign  nxt_sf_reg = ({16'h0, sf_reg[6:0]} << sf_len) | sf_val; // 把数据拼到一起 （旧数据剩余部分(最多7bit)+新来的数据）
assign  nxt_bits_cnt= {2'b0, bits_cnt[2:0]} + sf_len;  //discard already muxed out 8 or 16 bits. sg寄存器里，有多少数据

assign  nxt_bits_cnt_add0   = nxt_bits_cnt + add0_flag; //触发某些条件，需要多输出一个bit 的0

// 一个周期最多写进 寄存器16 bit
always @(*) begin
    if(sf_sta == s_sf_h) begin
        sf_len      = (big_16bit)? ((fifo_len[5])? 'd16 : {1'b0, fifo_len[3:0]}) : fifo_len[4:0];
        sf_val      = (big_16bit)? 16'h0 : fifo_val; // 如果一组数据超过16bit，前面的部分必为0 
    end else begin
        sf_len      = 'd16;      // 因为是低16 bit 肯定全部输出
        sf_val      = fifo_val;
    end
end

// 位移寄存器的状态 fifo 输出往里面喂数据，最多一个syntax 进来32bit，但最多只能写16bit
always @(posedge clk or negedge rstn)
if(!rstn)
    sf_sta  <= s_sf_h;
else if(sof)
    sf_sta  <= s_sf_h;
else if((!s0_vld) | s0_ack) begin
    case(sf_sta)
    s_sf_h: begin
        if(!fifo_empty) begin
            if(nxt_bits_cnt_add0[4:3] == 2'b11) //正好24bits  将要溢出
                sf_sta  <= s_out_2;
            else if(big_16bit)  // 超过16bit 先输出高位，进入状态s_sf_l
                sf_sta  <= s_sf_l; 
        end
    end
    
    s_sf_l: begin
        if(nxt_bits_cnt_add0[4:3] == 2'b11)     //24bits 如果把低位16bit输入 将要溢出
            sf_sta  <= s_out_2; 
        else                                    // 如果不溢出，输出后回到默认态
            sf_sta  <= s_sf_h;
    end
    
    s_out_2:begin
        if(back_flag)                        // 如果是是倒大盆水倒一半被打断的，得回 s_sf_l 把剩下的一半倒完
            sf_sta  <= s_sf_l;
        else                                 // 如果不是，输出后回到默认态
            sf_sta  <= s_sf_h;
    end
    endcase
end

always @(posedge clk or negedge rstn)        // 如果是输出到一半，sf_reg 将要溢出，需要做一个标记，以便回退到把剩下一半输出
if(!rstn)
    back_flag   <= 1'b0;
else if((sf_sta == s_sf_h) && (!fifo_empty)) begin
    if((nxt_bits_cnt_add0[4:3] == 2'b11) && big_16bit)
        back_flag   <= 1'b1;
    else
        back_flag   <= 1'b0;
end else if(sf_sta == s_sf_l)
    back_flag   <= 1'b0;


//add 1'b0 after 8'hff byte, at most add 1bit for a syntax
// 注意 每一次切割是从高位开始的
always @(*) begin
    add0_flag = 0;
    if(!head_time) begin
        //bits already in sf_reg can't have 8'hff byte
        //shift in bits pattern:
        //runcnt: 1'b1 or {1'b0, 0~15'hxxxx}
        //golomb: when (err_map_sf < limit):  {{err_map_sf[4:0]{1'b0}}, 1'b1, 0~7'hxx}, err_map_sf[4:0] can be 0;
        //        when (err_map_sf >= limit): {{limit{1'b0}}, 1'b1, 8'hxx}; limit in the rang of [7,23];
        //
        //So:1: each shift in can at most output one 8'hff and insert 1'b0;
        //   2: the first location result 8'hff is: shift in is: {1'b1, 7'hxx} or {1'b0, 0~15'hxxxx};
        // 注意 已经在寄存器里的位，不可能包含0xff，每次移入新数据，也最多只会产生一个0xff。
        
        //may                                   golomb or runcnt code           runcnt code
        if((nxt_sf_reg[14:7] == 8'hff) && ((nxt_bits_cnt == 'd15) || (nxt_bits_cnt == 'd23))) begin
            nxt_sf_reg_add0 = {nxt_sf_reg[22:7], 1'b0, nxt_sf_reg[6:0]};
            add0_flag = 1;
        end else if((nxt_sf_reg[13:6] == 8'hff) && ((nxt_bits_cnt == 'd14) || (nxt_bits_cnt == 'd22))) begin
            nxt_sf_reg_add0 = {nxt_sf_reg[22:6], 1'b0, nxt_sf_reg[5:0]};
            add0_flag = 1;
        end else if((nxt_sf_reg[12:5] == 8'hff) && ((nxt_bits_cnt == 'd13) || (nxt_bits_cnt == 'd21))) begin
            nxt_sf_reg_add0 = {nxt_sf_reg[22:5], 1'b0, nxt_sf_reg[4:0]};
            add0_flag = 1;
        end else if((nxt_sf_reg[11:4] == 8'hff) && ((nxt_bits_cnt == 'd12) || (nxt_bits_cnt == 'd20))) begin
            nxt_sf_reg_add0 = {nxt_sf_reg[22:4], 1'b0, nxt_sf_reg[3:0]};
            add0_flag = 1;
        end else if((nxt_sf_reg[10:3] == 8'hff) && ((nxt_bits_cnt == 'd11) || (nxt_bits_cnt == 'd19))) begin
            nxt_sf_reg_add0 = {nxt_sf_reg[22:3], 1'b0, nxt_sf_reg[2:0]};
            add0_flag = 1;
        //may                                   golomb or runcnt code           runcnt code
        end else if((nxt_sf_reg[9:2] == 8'hff) && ((nxt_bits_cnt == 'd10)  || (nxt_bits_cnt == 'd18))) begin
            nxt_sf_reg_add0 = {nxt_sf_reg[22:2], 1'b0, nxt_sf_reg[1:0]};
            add0_flag = 1;
        //may                                   golomb or runcnt code           golomb or runcnt code
        end else if((nxt_sf_reg[8:1] == 8'hff) && ((nxt_bits_cnt == 'd9)   || (nxt_bits_cnt == 'd17))) begin
            nxt_sf_reg_add0 = {nxt_sf_reg[22:1], 1'b0, nxt_sf_reg[0:0]};
            add0_flag = 1;
        //may                                   golomb or runcnt code           golomb or runcnt code
        end else if((nxt_sf_reg[7:0] == 8'hff) && ((nxt_bits_cnt == 'd8)   || (nxt_bits_cnt == 'd16))) begin
            nxt_sf_reg_add0 = {nxt_sf_reg[22:0], 1'b0};
            add0_flag = 1;
        end else begin
            nxt_sf_reg_add0 = {1'b0, nxt_sf_reg[22:0]};
            add0_flag = 0;
        end
    end else begin
        nxt_sf_reg_add0 = {1'b0, nxt_sf_reg[22:0]}; //如果没有额外增加1bit，最多23bit 有效
        add0_flag = 0;
    end
end

always @(posedge clk)   // or negedge rstn) // 更新sf_reg 
if(((!s0_vld) | s0_ack) && sf_bits)
    sf_reg  <= nxt_sf_reg_add0;

always @(posedge clk)   // or negedge rstn)
if(sof)
    bits_cnt    <= 'd0;
else if((!s0_vld) | s0_ack) begin
    if(sf_bits)
        bits_cnt    <= nxt_bits_cnt_add0;
    else if(sf_sta == s_out_2)
        bits_cnt    <= bits_cnt - 'd16;
end

always @(posedge clk or negedge rstn)
if(!rstn)
    s0_vld  <= 1'b0;
else if((!s0_vld) || s0_ack) begin
    if(sf_bits)
        s0_vld  <= 1'b1;
    else if(sf_sta == s_out_2)
        s0_vld  <= 1'b1;
    else if(s0_ack)
        s0_vld  <= 1'b0;
end

always @(posedge clk or negedge rstn)
if(!rstn)
    sf_eof  <= 1'b0;
else if(fifo_pop && fifo_flush)
    sf_eof  <= 1'b1;
else if(stream_eof)
    sf_eof  <= 1'b0;

//--- 2.2 stage_1: mux out valid higer 16 bits

reg     [15:0]  mux_16bits  ;

always @(*) begin
    if(bits_cnt[4]) begin       //24~16
        case(bits_cnt[3:0])
        'd0:    mux_16bits = sf_reg[0 +: 16];   //16bits 的情况
        'd1:    mux_16bits = sf_reg[1 +: 16];   //17bits 的情况
        'd2:    mux_16bits = sf_reg[2 +: 16];   //18bits 的情况
        'd3:    mux_16bits = sf_reg[3 +: 16];   //19bits 的情况
        'd4:    mux_16bits = sf_reg[4 +: 16];   //20bits 的情况
        'd5:    mux_16bits = sf_reg[5 +: 16];   //21bits 的情况
        'd6:    mux_16bits = sf_reg[6 +: 16];   //22bits 的情况
        'd7:    mux_16bits = sf_reg[7 +: 16];   //23bits 的情况
        default:mux_16bits = sf_reg[8 +: 16];   //24bits 的情况
        endcase
    end else begin              //15~8bits
        case(bits_cnt[3:0])
        'd8:    mux_16bits = sf_reg[0 +: 16];   //only use lower 8bits, sf_reg[7:0]
        'd9:    mux_16bits = sf_reg[1 +: 16];   //only use lower 8bits, sf_reg[8:1]
        'd10:   mux_16bits = sf_reg[2 +: 16];   //only use lower 8bits, sf_reg[9:2]
        'd11:   mux_16bits = sf_reg[3 +: 16];   //only use lower 8bits, sf_reg[10:3]
        'd12:   mux_16bits = sf_reg[4 +: 16];   //only use lower 8bits, sf_reg[11:4]
        'd13:   mux_16bits = sf_reg[5 +: 16];   //only use lower 8bits, sf_reg[12:5]
        'd14:   mux_16bits = sf_reg[6 +: 16];   //only use lower 8bits, sf_reg[13:6]
        default:mux_16bits = sf_reg[7 +: 16];   //only use lower 8bits, sf_reg[14:7]
        endcase
    end
end

assign  s0_ack = (!stream_vld) | stream_ack;

always @(posedge clk or negedge rstn)
if(~rstn)
    stream_vld  <= 1'b0;
else if(s0_vld && s0_ack && (bits_cnt[4:3] != 'd0))
    stream_vld  <= 1'b1;
else if(stream_ack)
    stream_vld  <= 1'b0;

always @(posedge clk)   // or negedge rstn)
if(s0_vld && s0_ack && (bits_cnt[4:3] != 'd0)) begin
    if(bits_cnt[4]) // 有16个以上的数据 可以进行输出2个
        stream  <= {mux_16bits[7:0], mux_16bits[15:8]};
    else
        stream  <= {mux_16bits[7:0], mux_16bits[7:0]};
end

always @(posedge clk)
if(s0_vld && s0_ack) begin
    stream_bytes    <= bits_cnt[4]; // 输出两个byte
end

assign  stream_len  = bits_acc[3 +: 24]; //总的累计输出长度

always @(posedge clk or negedge rstn)  // 一帧结束标志
if(~rstn)
    stream_eof  <= 1'b0;
else if(sof)
    stream_eof  <= 1'b0;
else if(s0_vld && s0_ack && sf_eof)
    stream_eof  <= 1'b1;

// 每次会输出8/16 bit，剩余未输出的数，最多是7bit。 若出现nxt_bits_cnt_add0[4:3] == 2'b11 即24bit 时，将停止读数据，先进行输出

endmodule