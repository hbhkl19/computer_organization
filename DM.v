`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2024/12/13 12:37:39
// Design Name: 
// Module Name: dm
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


//宏定义
`define dm_word 3'b000
`define dm_halfword 3'b001
`define dm_halfword_unsigned 3'b010
`define dm_byte 3'b011
`define dm_byte_unsigned 3'b100

module dm( 
input		clk, // 分频后时钟,CLK_CPU
    input       rstn,
    input 		DMWr,  //write signal
    input [9:0]		addr,
    input [31:0]	din,
    input [2:0]		DMType,
    input [15:0]    sw_i, 
    output reg [31:0]   dout
); 
integer i;
reg [7:0]dmem[511:0];
always@(posedge clk or negedge rstn)begin
if(!rstn&&!sw_i[1]) begin
for(i=0;i<=511;i=i+1)
dmem[i]=0;
end
else if(DMWr==1'b1&&sw_i[1]==1'b0)begin
case(DMType)
`dm_byte:dmem[addr]<=din[7:0];
`dm_halfword:begin
dmem[addr]<=din[7:0];
dmem[addr+1]<=din[15:8];end
`dm_word:begin
dmem[addr]<=din[7:0];
dmem[addr+1]<=din[15:8];
dmem[addr+2]<=din[23:16];
dmem[addr+3]<=din[31:24];end
endcase
end
end

always@(*)begin
case(DMType)
 `dm_byte: dout={{24{dmem[addr][7]}},dmem[addr][7:0]};
 `dm_halfword: dout={{16{dmem[addr+1][7]}},dmem[addr+1][7:0],dmem[addr][7:0]};
 `dm_word: dout={dmem[addr+3][7:0],dmem[addr+2][7:0],dmem[addr+1][7:0],dmem[addr][7:0]};
 `dm_byte_unsigned:dout={{24{1'b0}},dmem[addr][7:0]};
 `dm_halfword_unsigned:dout={{16{1'b0}},dmem[addr+1][7:0],dmem[addr][7:0]}; 
 default:dout={dmem[addr+3][7:0],dmem[addr+2][7:0],dmem[addr+1][7:0],dmem[addr][7:0]};
endcase
end

endmodule

