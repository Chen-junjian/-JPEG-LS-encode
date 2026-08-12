module dpram_generic(
                    clka,
                    ena,
                    wea,
                    dina,
                    addra,         
                    clkb,
                    enb,
                    addrb,
                    doutb 
                    );    

parameter ADDR_BITS = 7;
parameter ADDR_AMOUNT = 128;
parameter DATA_BITS = 32;

//-- write port
input clka;
input ena;
input wea;
input [DATA_BITS-1:0]dina;
input [ADDR_BITS-1:0]addra;                    
//-- read port
input clkb;                         
input enb;
input [ADDR_BITS-1:0]addrb;
output [DATA_BITS-1:0]doutb;

reg [DATA_BITS-1:0]doutb;
reg [DATA_BITS-1:0]mem[0 : ADDR_AMOUNT-1];


always@(posedge clkb)                //-- read 
begin
   if(enb==1'b1)
      doutb<=mem[addrb];
end

always@(posedge clka)                //-- write
begin
   if(ena==1'b1) 
      begin
         if(wea==1'b1)
            mem[addra]<=dina;
      end 
end

endmodule

