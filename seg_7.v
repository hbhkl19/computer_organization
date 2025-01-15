`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2024/12/13 12:34:44
// Design Name: 
// Module Name: seg7
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


module seg7(
input clk,
input rstn,
input disp_mode,
input [63:0]i_data,
output [7:0]o_seg,
output [7:0]o_sel
   );
 //  reg [31:0]i_data=32'b0111_0110_0101_0100_0011_0010_0001_0000;
   reg [14:0] cnt;
   wire seg7_clk;
   
   //分频
   always@(posedge clk or negedge rstn)begin
   if(!rstn)
    cnt<=0;
    else
    cnt<=cnt+1'b1;
   end 
   assign seg7_clk=cnt[14];
   
   //选地址
   reg [2:0]seg7_addr;
   always@(posedge seg7_clk or negedge rstn)begin
   if(!rstn)
   seg7_addr<=0;
   else
   seg7_addr<=seg7_addr+1'b1;
   end
   
   //对地址译码
   reg[7:0] o_sel_r;
   always@(*) 
   case(seg7_addr)
    7:o_sel_r=8'b01111111;
    6:o_sel_r=8'b10111111;   
    5:o_sel_r=8'b11011111;
    4:o_sel_r=8'b11101111;
    3:o_sel_r=8'b11110111;
    2:o_sel_r=8'b11111011;
    1:o_sel_r=8'b11111101;
    0:o_sel_r=8'b11111110;
    endcase
reg [63:0]i_data_r;
    always@(posedge clk or negedge rstn)
    if(!rstn)
    i_data_r<=0;
    else
    i_data_r<=i_data;
    
    reg [7:0]seg_data_r;
    always@(*)
    if(disp_mode==1'b0)
    begin
    case(seg7_addr)
        0:seg_data_r=i_data_r[3:0];
        1:seg_data_r=i_data_r[7:4];
        2:seg_data_r=i_data_r[11:8];
        3:seg_data_r=i_data_r[15:12];
        4:seg_data_r=i_data_r[19:16];
        5:seg_data_r=i_data_r[23:20];
        6:seg_data_r=i_data_r[27:24];
        7:seg_data_r=i_data_r[31:28];
    endcase
    end
    else
    begin
    case(seg7_addr)
    0:seg_data_r=i_data_r[7:0];
    1:seg_data_r=i_data_r[15:8];
    2:seg_data_r=i_data_r[23:16];
    3:seg_data_r=i_data_r[31:24];
    4:seg_data_r=i_data_r[39:32];
    5:seg_data_r=i_data_r[47:40];
    6:seg_data_r=i_data_r[55:48];
    7:seg_data_r=i_data_r[63:56];
    endcase
    end
    //对数值译码
    reg[7:0]o_seg_r;
    always@(posedge clk or negedge rstn)
    if(!rstn)
    o_seg_r<=8'hff;
    else if(disp_mode==1'b0)begin
        case(seg_data_r)
        4'h0:o_seg_r<=8'hC0;
        4'h1:o_seg_r<=8'hF9;
        4'h2:o_seg_r<=8'hA4;
        4'h3:o_seg_r<=8'hB0;
        4'h4:o_seg_r<=8'h99;
        4'h5:o_seg_r<=8'h92;
        4'h6:o_seg_r<=8'h82;
        4'h7:o_seg_r<=8'hF8;
        4'h8:o_seg_r<=8'h80;
        4'h9:o_seg_r<=8'h90;
        4'hA:o_seg_r<=8'h88;
        4'hB:o_seg_r<=8'h83;
        4'hC:o_seg_r<=8'hC6;
        4'hD:o_seg_r<=8'hA1;
        4'hE:o_seg_r<=8'h86;
        4'hF:o_seg_r<=8'h8E;
        default:o_seg_r<=8'hFF;
        endcase
        end
        else
        begin  
        o_seg_r<=seg_data_r;
        end
     assign o_sel=o_sel_r;
     assign o_seg=o_seg_r;
endmodule
