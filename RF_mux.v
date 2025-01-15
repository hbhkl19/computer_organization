`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2024/12/16 10:59:46
// Design Name: 
// Module Name: rf_mux
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
`define WDSel_FromALU 2'b00
`define WDSel_FromMEM 2'b01
`define WDSel_FromPC 2'b10

module rf_mux(
input [1:0]WDSel,   // (register) write data selection  (MemtoReg)
input[31:0] dout,
input[31:0] aluout,
input[31:0] PC_out,
output reg [31:0] WD
    );
    always @(*)
begin
	case(WDSel)
		`WDSel_FromALU: WD<=aluout;
		`WDSel_FromMEM: WD<=dout;
		`WDSel_FromPC: WD<=PC_out+1;    //疑问：应该加4还是加1???
	endcase

end

endmodule
