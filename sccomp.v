`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2024/11/24 08:43:37
// Design Name: 
// Module Name: test
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

//定义从Mem2Reg的Mux写入的信号
`define WDSel_FromALU 2'b00
`define WDSel_FromMEM 2'b01
`define WDSel_FromPC 2'b10

// NPC control signal
`define NPC_PLUS4   3'b000
`define NPC_BRANCH  3'b001
`define NPC_JUMP    3'b010
`define NPC_JALR 3'b100

//ALUOp的定义
`define ALUOp_nop 5'b00000
`define ALUOp_lui 5'b00001
`define ALUOp_auipc 5'b00010
`define ALUOp_add 5'b00011
`define ALUOp_sub 5'b00100
`define ALUOp_bne 5'b00101
`define ALUOp_blt 5'b00110
`define ALUOp_bge 5'b00111
`define ALUOp_bltu 5'b01000
`define ALUOp_bgeu 5'b01001
`define ALUOp_slt 5'b01010
`define ALUOp_sltu 5'b01011
`define ALUOp_xor 5'b01100
`define ALUOp_or 5'b01101
`define ALUOp_and 5'b01110
`define ALUOp_sll 5'b01111
`define ALUOp_srl 5'b10000
`define ALUOp_sra 5'b10001



module sccomp(clk,rstn,sw_i,disp_seg_o,disp_an_o);
input clk;
input rstn;
input [15:0]sw_i;
output [7:0]disp_seg_o,disp_an_o;

//========================第一部分：快慢时钟分频======================
reg[31:0]clkdiv;
wire Clk_CPU;   
always @ (posedge clk or negedge rstn)begin
if(!rstn) clkdiv<=0;
else clkdiv<=clkdiv+1'b1;
end
assign Clk_CPU=(sw_i[15])?clkdiv[27]:clkdiv[25];
//===========================分频================================


reg [63:0]display_data;


//=====================第二部分：图形变换=======================
reg [5:0]led_data_addr;
reg [63:0]led_disp_data;
parameter LED_DATA_NUM=19;
reg [63:0]LED_DATA[18:0];
initial begin
LED_DATA[0]=64'hC6F6F6F0C6F6F6F0;
LED_DATA[1]=64'hF9F6F6CFF9F6F6CF;
LED_DATA[2]=64'hFFC6F0FFFFC6F0FF;
LED_DATA[3]=64'hFFC0FFFFFFC0FFFF;
LED_DATA[4]=64'hFFA3FFFFFFA3FFFF;
LED_DATA[5]=64'hFFFFA3FFFFFFA3FF;
LED_DATA[6]=64'hFFFF9CFFFFFF9CFF;
LED_DATA[7]=64'hFF9EBCFFFF9EBCFF;
LED_DATA[8]=64'hFF9CFFFFFF9CFFFF;
LED_DATA[9]=64'hFFC0FFFFFFC0FFFF;
LED_DATA[10]=64'hFFA3FFFFFFA3FFFF;
LED_DATA[11]=64'hFFA7B3FFFFA7B3FF;
LED_DATA[12]=64'hFFC6F0FFFFC6F0FF;
LED_DATA[13]=64'hF9F6F6CFF9F6F6CF;
LED_DATA[14]=64'h9EBEBEBC9EBEBEBC;
LED_DATA[15]=64'h2737373327373733;
LED_DATA[16]=64'h505454EC505454EC;
LED_DATA[17]=64'h744454F8744454F8;
LED_DATA[18]=64'h0062080000620800;
end
//输出图形变换
always @(posedge Clk_CPU or negedge rstn) begin 
  if(!rstn) begin 
    led_data_addr <= 6'd0; 
    led_disp_data <= 64'b1; 
  end
  else if(sw_i[0] == 1'b1) begin 
    if(led_data_addr == LED_DATA_NUM) 
    begin 
      led_data_addr <= 6'd0;
      led_disp_data <= 64'b1;
    end
    led_disp_data <= LED_DATA[led_data_addr]; 
    led_data_addr <= led_data_addr + 1'b1; 
  end
  else led_data_addr <= led_data_addr; 
end
//==================================图形变换==========================

//==============================第三部分：数码显示===============
wire[31:0]instr;

reg[31:0] reg_data;

reg[31:0] alu_disp_data;

reg[31:0]dmem_data;

always @(sw_i) begin
if(sw_i[0]==0)begin
case(sw_i[14:11])
4'b1000:display_data=instr;
4'b0100:display_data=reg_data;
4'b0010:display_data=alu_disp_data;
4'b0001:display_data=dmem_data;
4'b0011:display_data=immout;
4'b0111:display_data=rom_addr;
default:display_data=instr;
endcase
end
else display_data=led_disp_data;
end
//=================================数码显示==============================


//===================================第四部分：例化rom取指令============================
parameter IM_CODE_NUM = 24;//rom地址最大值
reg[31:0] rom_addr;
dist_mem_im U_IM(          //例化rom
.a(rom_addr),
.spo(instr)
);
//==========================================================

//===============================第五部分,Decode,对变量赋初值==================
wire [6:0]Op;  
assign Op = instr[6:0]; //opcode
wire [6:0]Funct7;
assign Funct7 = instr[31:25];   //funcnt7
wire [2:0]Funct3;
assign Funct3 = instr[14:12];   //funct3
wire [4:0]rs1;
assign rs1 = instr[19:15];  // rs1
wire [4:0]rs2 ;
assign rs2= instr[24:20];  // rs2
wire [4:0]rd;
assign rd = instr[11:7];  // rd
wire signed [11:0]iimm;      
assign iimm = instr[31:20]; //i型指令的立即数解析方式生成的立即数,jalr,ld,addi
wire [11:0]simm;
assign simm = {instr[31:25],instr[11:7]};   //s型指令立即数解析方式生成的立即数
wire [4:0] iimm_shamt;
assign iimm_shamt = instr[24:20];   //i形指令移位立即数,bne,beq
wire [19:0] jimm;
assign jimm = {instr[31],instr[19:12],instr[20],instr[30:21]};  //UJ型指令立即数,jal
wire [11:0] bimm;
assign bimm = {instr[31],instr[7],instr[30:25],instr[11:8]};    //SB型指令立即数
wire [19:0] uimm;
assign uimm = instr[31:12]; //U型指令立即数,lui
//====================================================================

//===================第六部分，控制信号例化=================== 
    wire[7:0] Zero; //alu输出的两个值之一，用于branch
    wire RegWrite;//用于控制写信号
    wire MemWrite;//是否写入Mem
    wire[5:0]EXTOp;//立即数生成信号
    wire[4:0] ALUOp;//ALUOp
    wire ALUSrc;//ALUSrc
    wire [2:0] DMType;//Mem读取/写入方式：字节/半字...
    wire [1:0]WDSel;//控制信号,写回寄存器
    wire [2:0]NPCOp;//NPCOp下一条ROMadddress
control U_control(
    .Op(Op),
    .Funct3(Funct3),
    .Funct7(Funct7),
    .Zero(Zero),
    .RegWrite(RegWrite),
    .MemWrite(MemWrite),
    .EXTOp(EXTOp),
    .ALUOp(ALUOp),
    .NPCOp(NPCOp),
    .ALUSrc(ALUSrc),
    .DMType(DMType),
    .WDSel(WDSel)
    );
//=========================================================

//================第七部分，立即数生成===============
  wire [31:0]immout;//立即数生成输出的立即数
  wire [31:0]immout_new;
  assign immout_new={immout[31],immout[31],immout[31:2]};
EXT U_EXT(
    .iimm_shamt(iimm_shamt),
    .iimm(iimm),
    .simm(simm),
    .bimm(bimm),
    .uimm(uimm),
    .jimm(jimm),
    .EXTOp(EXTOp),
    .immout(immout)
); 
//==============================================


//================第八部分，RF===================
wire[31:0]RD1;//第一个读端口读出来的数据
wire[31:0]RD2;//第二个读端口读出来的数据
wire[31:0] WD; //写入寄存器的数据
reg [4:0] reg_addr; //寄存器地址
//RF的例化
RF U_RF(
.clk(Clk_CPU),
.rst(rstn),
.RFWr(RegWrite),
.A1(rs1),
.A2(rs2),
.A3(rd),
.WD(WD),
.sw_i(sw_i),
.RD1(RD1),
.RD2(RD2)
);
 //reg_addr增加+读出对应rf的值
 always@(posedge Clk_CPU or negedge rstn) 
      begin
         if(!rstn) 
         begin reg_addr = 5'd0; reg_data = U_RF.rf[reg_addr];end
         else if(sw_i[13]==1'b1) begin 
         begin
            reg_addr = reg_addr + 1'b1;
            reg_data = U_RF.rf[reg_addr]; 
          end
             if (reg_addr == 31)
              begin reg_addr = 5'd0;end
           end
         else reg_addr = reg_addr;
      end
 //============================================

//==============第九部分ALU===================
    wire signed [31:0]A;    //ALU第一个输入
    wire signed [31:0]B;    //ALU第二个输入
    wire signed[31:0] aluout;   //ALU的输出
    reg[2:0] alu_addr = 0; //用于选择输出ALU的不同量
    //选择ALU的输入，用mux搞定RD2
    assign A=RD1;
alu_mux U_alu_mux(
.immout(immout),
.RD2(RD2),
.ALUSrc(ALUSrc),
.B(B)
);
//alu例化
alu U_alu(
.clk(clk), 
.rstn(rstn), 
.A(A), 
.B(B), 
.rom_addr(rom_addr),
.ALUOp(ALUOp), 
.C(aluout), 
.Zero(Zero)
);
 //循环显示ALU的内容，选择ALU的输出数据
always@(posedge Clk_CPU or negedge rstn)
begin
    if(!rstn)alu_addr=3'b000;
    else if(sw_i[12]==1'b1)
    begin
    if(alu_addr==3'b100) begin alu_addr=3'b000;alu_disp_data=32'hffffffff;end
    else begin
   case(alu_addr)
     3'b001:alu_disp_data=U_alu.A;
     3'b010:alu_disp_data=U_alu.B;
     3'b011:alu_disp_data=U_alu.C;
     3'b100:alu_disp_data=U_alu.Zero;
    default:alu_disp_data=32'hFFFFFFFF;
    endcase
    alu_addr=alu_addr+1'b1;
    end
    end
    else alu_disp_data=alu_disp_data;
end
//===============================================

//=================================第十部分：dm==================
wire [9:0]dm_addr;  
assign dm_addr=aluout;  //Mem的地址，由ALU计算得出
wire [31:0]dm_din;
assign dm_din=RD2;  //输入Mem的数据，store指令时，用第二个读端口读出来的数据
wire [31:0]dm_dout; 
reg [7:0]dmem_addr=0;   //读dmem的值
parameter DM_DATA_NUM=16;   //读dmem的最大地址
       
//dm实例化
      dm U_DM(
            .clk(Clk_CPU),
            .rstn(rstn),
            .DMWr(MemWrite),
            .addr(dm_addr),
            .din(dm_din),
            .DMType(DMType),
            .sw_i(sw_i),
            .dout(dm_dout)
      );
 //循环显示dm内容：只显示前面16个
      always@(posedge Clk_CPU or posedge rstn)
      begin
      if(!rstn)begin dmem_addr=7'b0;dmem_data=U_DM.dmem[dmem_addr][7:0];end
      else if (sw_i[11]==1'b1)
            begin
            dmem_addr = dmem_addr+1'b1;
            dmem_data = U_DM.dmem[dmem_addr][7:0];
            dmem_data = {dmem_addr,{dmem_data[27:0]}};  //显示是第几号存储单元
            if (dmem_addr==DM_DATA_NUM) begin
            dmem_addr = 7'b0;
            dmem_data = 32'hffffffff;end
            end
      end 

 //选择输入rf的值 
rf_mux U_RF_mux(
        .WDSel(WDSel),
        .dout(dm_dout),
        .PC_out(rom_addr),
        .aluout(aluout),
        .WD(WD)
);
//=========================================


//===========第十一部分，ROM地址的更新，写回PC======
always@(posedge Clk_CPU or negedge rstn)begin
        if(!rstn) begin if(sw_i[1]==1'b0)rom_addr = 32'b0;end   //清零
        else 
        begin 
            if(sw_i[1] == 1'b0) //正常模式          
                begin
                case(NPCOp)
                `NPC_PLUS4: rom_addr = (rom_addr +1'b1)%(IM_CODE_NUM);                            
                `NPC_BRANCH:begin 
                            case(ALUOp)
                            `ALUOp_sub:if(Zero==1) rom_addr =(rom_addr+immout_new)%(IM_CODE_NUM); else rom_addr = (rom_addr +1'b1)%(IM_CODE_NUM);
                            `ALUOp_bne:if(Zero==0) rom_addr =(rom_addr+immout_new)%(IM_CODE_NUM); else rom_addr = (rom_addr +1'b1)%(IM_CODE_NUM);
                            `ALUOp_blt:if(aluout<0) rom_addr =(rom_addr+immout_new)%(IM_CODE_NUM); else rom_addr = (rom_addr +1'b1)%(IM_CODE_NUM);
                            `ALUOp_bge:if(aluout>=0) rom_addr =(rom_addr+immout_new)%(IM_CODE_NUM); else rom_addr = (rom_addr +1'b1)%(IM_CODE_NUM);
                            `ALUOp_bltu:if(aluout==1) rom_addr =(rom_addr+immout_new)%(IM_CODE_NUM); else rom_addr = (rom_addr +1'b1)%(IM_CODE_NUM);
                            `ALUOp_bgeu:if(aluout==1) rom_addr =(rom_addr+immout_new)%(IM_CODE_NUM); else rom_addr = (rom_addr +1'b1)%(IM_CODE_NUM);
                            default:rom_addr = (rom_addr +1'b1)%(IM_CODE_NUM);
                            endcase
                            end
                `NPC_JUMP:rom_addr = (rom_addr+immout_new)%(IM_CODE_NUM); 
                `NPC_JALR: rom_addr = (U_RF.rf[rs1]+immout_new)%(IM_CODE_NUM);
                endcase
               end
            else if(sw_i[1]==1'b1)rom_addr = rom_addr;  //调试模式
        end
    end
//===================================================


//===========第十二部分,数码管的实例化===========
seg7 my_seg7(
.clk(clk),
.rstn(rstn),
.i_data(display_data),
.disp_mode(sw_i[0]),
.o_seg(disp_seg_o),
.o_sel(disp_an_o)
);



endmodule

