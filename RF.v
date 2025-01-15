`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2024/12/13 12:30:30
// Design Name: 
// Module Name: RF
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


module RF(
input clk,
input rst,
input RFWr,
input [15:0]sw_i,
input [4:0]A1,A2,A3,
input [31:0]WD,
output wire[31:0]RD1,RD2
);
reg [31:0] rf[31:0];

integer i;
//初始化
initial begin
  for(i=0;i<=31;i=i+1)
            rf[i] =i;
   end
always@(posedge clk,negedge rst)begin
if(!rst&&~sw_i[1])begin
for(i=0;i<32;i=i+1)
rf[i]=i; end
else 
if(RFWr&&(A3!=0)&&~sw_i[1])
rf[A3]<=WD;

end

assign RD1=(A1!=0)?rf[A1]:0;
assign RD2=(A2!=0)?rf[A2]:0;

endmodule
