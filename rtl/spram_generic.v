module spram_generic(
                    clk,
                    en,
                    we,
                    addr,
                    din,
                    
                    dout
                    );    

parameter ADDR_BITS = 7;
parameter ADDR_AMOUNT = 128;
parameter DATA_BITS = 32;

input clk;
input en;
input we;
input [ADDR_BITS-1:0]addr;
input [DATA_BITS-1:0]din;
output [DATA_BITS-1:0]dout;

reg [DATA_BITS-1:0]dout;
reg [DATA_BITS-1:0]mem[0 : ADDR_AMOUNT-1];

always@(posedge clk)
begin
	if(en) begin
        if(we==1'b1) begin
   	    	mem[addr]<=din; 
        	dout<=din; 
        end else begin
        	dout<=mem[addr]; 
        end
    end     
end


endmodule

