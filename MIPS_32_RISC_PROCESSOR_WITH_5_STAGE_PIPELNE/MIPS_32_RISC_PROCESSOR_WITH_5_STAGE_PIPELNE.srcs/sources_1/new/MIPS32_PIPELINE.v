`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Raju Katala
// 
// Create Date: 18/10/2021 07:08:34 PM
// Design Name: 
// Module Name: MIPS32_PIPELINE
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

//********************************** Main Block **********************************************************************

module MIPS32_PIPELINE(clk, rest);
 input clk, rest;
 wire PCSrc, jump, RegDst, branch, memRead, memtoReg, memWrite, aluSrc, regWrite, zero, zeroOut, ex_memZero;
 wire [31:0] pcout, instruction, BranchAddress, jumpAddress, if_idInstruction, if_idPcout, writeData, signEx, ReadData1,
             ReadData2, id_exPcout, id_ex_ReadData1, id_exReadData2, id_exSignEx, addOut, alu_out, ex_memAddout, ex_memAluOut,
             ex_memReadData2, data_out, mem_wbDataOut, mem_wbAluOut;    
 wire [4:0] writeAddress, Instr15_11, Instr20_16, id_exInstr15_11, id_exInstr20_16,  muxout,
            ex_memMuxout, mem_wbMuxout; 
 wire [5:0]  Instr5_0, id_exInstr5_0; 
 wire [1:0] WB, id_exWB,  ALUop, ex_memWB;
 wire [2:0] M, id_exM; 
 wire [3:0]  EX;
 IF     IF_stage(pcout, instruction, BranchAddress, jumpAddress, PCSrc, jump, clk, rest);
 
 Latch  IF_ID(if_idInstruction, if_idPcout, pcout, instruction, clk);
 
 ID     ID_stage(jumpAddress, signEx, ReadData1, ReadData2, Instr15_11, Instr20_16, Instr5_0, jump, WB, M, EX, if_idInstruction,
                  if_idPcout, writeData, regWrite, writeAddress, clk,rest);
                  
 Latch1 ID_EX(id_exPcout, id_ex_ReadData1, id_exReadData2, id_exSignEx, id_exInstr15_11, id_exInstr20_16, id_exInstr5_0,
               RegDst, aluSrc, ALUop, id_exWB, id_exM, signEx, ReadData1, ReadData2, if_idPcout, Instr15_11, Instr20_16,
               Instr5_0, WB, M, EX, clk);
               
 EX  EX_stge(addOut, alu_out, zero, muxout, id_exPcout, id_ex_ReadData1, id_exReadData2, id_exSignEx, id_exInstr15_11,
                  id_exInstr20_16, id_exInstr5_0, RegDst, aluSrc, ALUop, clk);
                  
 Latch2 EX_MEM(BranchAddress, ex_memAluOut, ex_memReadData2, zeroOut, ex_memWB, ex_memMuxout, branch, memWrite, memRead,
                id_exWB, id_exM, addOut, alu_out, zero, muxout, id_exReadData2, clk);
                
 MEM    MEM_stage(data_out, PCSrc, branch, memWrite, memRead, zeroOut, ex_memAluOut, ex_memReadData2, clk);
 
 Latch3 MEM_WB(memtoReg, regWrite, writeAddress, mem_wbDataOut, mem_wbAluOut, ex_memMuxout, ex_memAluOut, data_out,
                ex_memWB, clk);               
 Wb     WB_stage(writeData, memtoReg, mem_wbDataOut, mem_wbAluOut);
  
endmodule

//**************************** IF Stage **************************************************************************
module IF( output [31:0] pcout, instruction,
           input [31:0] BranchAddress, jumpAddress,
           input PCSrc, jump, clk, rest);
           
          wire [31:0] mux1, mux2, pc;
          mux MUX1(mux1, pcout, BranchAddress, PCSrc);
          mux MUX2(mux2, mux1, jumpAddress, jump);
          pc  PC(pc, mux2, clk, rest);
          IM  Instruction_memory(instruction, pc);
          add ADDER(pcout, pc);
endmodule
 
//************************** IF/ID Latch ***************************************************************************
module Latch( output reg [31:0] if_idInstruction, if_idPcout,
              input [31:0] pcout, instruction,
              input clk);
              always @(posedge clk)
                 begin
                   if_idInstruction <= instruction;
                   if_idPcout       <= pcout;
                 end
endmodule
  
            
//***************************32_bit MUX *****************************************************************************
module mux(output [31:0] muxout,
           input [31:0] addOut, Branch,
           input pcsrc );
           
           assign muxout = pcsrc?Branch:addOut;

endmodule

//***************************Program Counter *************************************************************
module pc(output reg [31:0] pcout,
          input [31:0] muxout,
          input clk,rest);
          always @(posedge clk)
             if(!rest)
                begin
                  pcout = 8'h0;
                end
              else
                begin
                  pcout <= muxout;
                end
endmodule
  
  
 //******************* Adder For Next Instruction*****************************************************************
module add(output [31:0] addOut,
           input [31:0] pcout
           );
assign addOut = pcout + 8'h00000004;
endmodule

//*********************************Instruction Memory **************************************************************
module IM(output reg [31:0] instruction,
          input [31:0] pcout);
          wire [31:0]mem[0:31];
 assign mem[0] = 32'b0000_0000_0100_0010_0001_1000_0010_0000; // fn1 R type
 assign mem[4] = 32'b0000_0000_0100_0010_0001_1000_0010_0001; // fn2 rR type
 assign mem[8] = 32'b0000_0100_0100_0010_0001_1000_0010_0010; // fn3 load word
 assign mem[12] = 32'b0000_0000_0100_0010_0001_1000_0010_0011; // fn4 R type
 assign mem[16] = 32'b0000_1000_0100_0010_0001_1000_0010_0100; // fn5 store word
 assign mem[20] = 32'b0000_0000_0100_0010_0001_1000_0010_0101; // fn6 R type
 assign mem[24] = 32'b0000_0000_0100_0010_0001_1000_0010_0110; // fn7 R type
 assign mem[28] = 32'b0000_1100_0100_0010_0001_1000_0010_0111; // fn8 branch eq
 assign mem[1]  = 32'b0000_0000_0100_0010_0001_1000_0010_0111;
 assign mem[2]  = 32'b0001_0000_0100_0010_0001_1000_0010_0111;  // jump
 assign mem[3]  = 32'b0000_0000_0100_0010_0001_1000_0010_0111;
 assign mem[5]  = 32'b0000_0000_0100_0010_0001_1000_0010_0111;
 assign mem[6]  = 32'b0001_0000_0100_0010_0001_1000_0010_0111;  // jump
 assign mem[7]  = 32'b0000_0000_0100_0010_0001_1000_0010_0111;
 assign mem[9]  = 32'b0000_0000_0100_0010_0001_1000_0010_0111;
 assign mem[10]  = 32'b0000_0000_0100_0010_0001_1000_0010_0111;
 assign mem[11]  = 32'b0000_0000_0100_0010_0001_1000_0010_0111;
 assign mem[13]  = 32'b0000_0000_0100_0010_0001_1000_0010_0111;
 assign mem[14]  = 32'b0000_0000_0100_0010_0001_1000_0010_0111;
 assign mem[15]  = 32'b0000_0000_0100_0010_0001_1000_0010_0111;
 assign mem[17]  = 32'b0000_0000_0100_0010_0001_1000_0010_0111;
 assign mem[18]  = 32'b0000_0000_0100_0010_0001_1000_0010_0111;
 assign mem[19]  = 32'b0000_0000_0100_0010_0001_1000_0010_0111;
 assign mem[21]  = 32'b0000_0000_0100_0010_0001_1000_0010_0111;
 assign mem[22]  = 32'b0000_0000_0100_0010_0001_1000_0010_0111;
 assign mem[23]  = 32'b0000_0000_0100_0010_0001_1000_0010_0111;
 assign mem[25]  = 32'b0000_0000_0100_0010_0001_1000_0010_0111;
 assign mem[26]  = 32'b0000_0000_0100_0010_0001_1000_0010_0111;
 assign mem[27]  = 32'b0000_0000_0100_0010_0001_1000_0010_0111;
 assign mem[29]  = 32'b0000_0000_0100_0010_0001_1000_0010_0111;
 assign mem[30]  = 32'b0000_0000_0100_0010_0001_1000_0010_0111;
 assign mem[31]  = 32'b0000_0000_0100_0010_0001_1000_0010_0111;
always @(*)
 begin
   instruction = mem[pcout%32];
 end 
endmodule

//******************************************* ID Stage ************************************************************* 
module ID( output [31:0] jumpAddress, signEx, ReadData1, ReadData2,
           output [4:0] Instr15_11, Instr20_16, 
           output [5:0] Instr5_0,
           output jump,
           output [1:0] WB,
           output [2:0] M,
           output [3:0]  EX,
           input [31:0] if_idInstruction, if_idPcout, writeData,
           input RegWrite,
           input [4:0]  writeAddress,
           input clk, rest);
           wire [27:0] ls;
           assign Instr15_11 = if_idInstruction[15:11],
                  Instr20_16 = if_idInstruction[20:16],
                  Instr5_0 =   if_idInstruction[5:0]; 
          control_path Main_Control(WB, M, EX, jump, if_idInstruction[31:26], clk);
          regbank      RegBank(ReadData1, ReadData2, if_idInstruction[25:21], if_idInstruction[20:16], writeData,
                                writeAddress, RegWrite, clk, rest);
          signex       SignEX( signEx, if_idInstruction[15:0]);
          assign  ls = if_idInstruction[25:0]<<2;
          assign  jumpAddress = {if_idInstruction[31:28],ls};
          
 endmodule 
 
//******************************************* ID/EX Latch **********************************************************
 module Latch1(output reg [31:0] id_exPcout, id_ex_ReadData1, id_exReadData2, id_exSignEx,
               output reg [4:0] id_exInstr15_11, id_exInstr20_16, 
               output reg [5:0] id_exInstr5_0,
               output reg RegDst, aluSrc,
                output reg [1:0]  ALUop, id_exWB,
                output reg [2:0] id_exM,
                input [31:0] signEx, ReadData1, ReadData2, if_idPcout,
                input [4:0] Instr15_11, Instr20_16, 
                input [5:0] Instr5_0,
                input [1:0] WB,
                input [2:0] M,
                input [3:0] EX,
                input clk);
                
                always @(posedge clk)
                  begin
                    id_exPcout <= if_idPcout;
                    id_ex_ReadData1 <=  ReadData1;
                    id_exReadData2 <= ReadData2;
                    id_exSignEx  <= signEx;
                    id_exInstr15_11 <= Instr15_11;
                    id_exInstr20_16 <=  Instr20_16;
                    id_exInstr5_0   <=   Instr5_0;
                    id_exWB      <=     WB;
                    id_exM     <= M;
                    ALUop   <=  EX[3:2];
                    RegDst  <= EX[0];
                    aluSrc   <= EX[1];
                  end       
 endmodule                
                
                         
//******************************************* 5_bit Mux ************************************************************
module mmux(output [4:0] muxout,
           input [4:0] rr2, wr,
           input RegDst );
           
           assign muxout = RegDst?wr:rr2;

endmodule

//********************************************Registor Bank ****************************************************
module regbank(readdata1, readdata2, rr1, rr2, writedata, muxout, regWrite, clk, rest);
input clk, regWrite, rest;
input [4:0] rr1, rr2, muxout; // Source and destination registers
input [31:0] writedata;
output [31:0] readdata1, readdata2;
integer k;
reg [31:0] regfile[0:31];
assign readdata1 = regfile [rr1];
assign readdata2 = regfile [rr2];
always @(posedge clk)
  begin
    if (rest)
     begin
      for (k=0; k<32; k=k+1) begin
        regfile[k] <= 0;
     end
  end
  else
   begin
     if (regWrite)
       regfile[muxout] <= writedata;
   end
  end
endmodule

//******************************************* Sign Extention *******************************************************
module signex(output [31:0] snx,
              input [15:0] se);
               
             assign snx = {4'h0000,se};
endmodule


 //***************************************** Execute Stage **********************************************************
 module EX(output [31:0] addOut, alu_out,
           output zero,
           output [4:0] muxout,
           input [31:0] id_exPcout, id_ex_ReadData1, id_exReadData2, id_exSignEx,
           input [4:0] id_exInstr15_11, id_exInstr20_16, 
           input [5:0] id_exInstr5_0,
           input RegDst, aluSrc,
           input [1:0] ALUop, 
           input clk);
           wire [31:0] m, s;
           wire [3:0] aluop;
           assign s = id_exSignEx<<2;
           mmux  MUX3(muxout, id_exInstr20_16, id_exInstr15_11, RegDst);
           mux   MUX4(m, id_exReadData2, id_exSignEx, aluSrc);
           adder ADDER1(addOut, s, id_exPcout);
           ALU  alu(alu_out, zero, id_ex_ReadData1, m, aluop);
           alucontrol ALUCONTROL(aluop, id_exInstr5_0, ALUop, clk);
 
 endmodule
 
//****************************************** Adder for Branch Pridection *******************************************
module adder(output [31:0] Branch,
             input [31:0] addOut,sl);
             
             assign Branch = sl + addOut;
             
 endmodule
 
 //****************************** Aerthemetic and Logic Unit ******************************************************
 module ALU( 
               //output
                 alu_out,
                 status,    // zero flag that can high whenever output is zero.
                 //input
                 a,
                 b,
                 alu_op 
 
               );    
 
 input [3:0]  alu_op;
 
 input [31:0]  a,
                b;
 
      
 
 output reg [31:0] alu_out;
 
 output  reg status; 
 
 parameter  [3:0] ADD = 4'b0000,  
 
                  SUB = 4'b0001,
 
                  AND = 4'b0010,
 
                  OR = 4'b0011,
 
                  NOT = 4'b0100,
 
                  MUL = 4'b0101,
 
                  DIV = 4'b0110,
 
                  RSHIFT = 4'b0111,
                  
                 LSHIFT = 4'b1000;
 
 always @ (alu_op or a or b)
 
 begin
 
  case (alu_op)
 
  ADD: begin                     //     use carry look ahead adder(Code Provide separate file)
 
    alu_out = a + b;
    status  = (alu_out == 32'b0);
 
   end
 
  SUB: begin                     // Use Alou_out = a+ 2' compliment 
 
    alu_out = a - b;
    status  = (alu_out == 32'b0);
 
   end
   AND: begin                
  
     alu_out = a & b;
  
     status  = (alu_out == 32'b0);
  
    end
    OR:  begin
    
        alu_out = a | b;
    
        status  = (alu_out == 32'b0);
    
       end
    NOT:  begin
       
           alu_out = ~(a+b);
       
           status  = (alu_out == 32'b0);
       
          end
  MUL: begin                                // use Booth Algoriths For siged integer(code provide separate file)
 
    alu_out = a* b;
 
    status  = (alu_out == 32'b0);
 
   end 
  DIV:  begin                              /// use Non-restoring Algorithms (code provide in separate file0
   
       alu_out = a/b;
   
       status  = (alu_out == 32'b0);
   
      end
  RSHIFT: begin
                      
           alu_out = a>>1;
                      
           status  = (alu_out == 32'b0);
                      
          end
  LSHIFT: begin
                               
           alu_out = a<<1;
                               
           status  = (alu_out == 32'b0);
                               
          end
  default: begin
 
    alu_out = 32'bzzzz;
 
    status  = 1'bz;
 
 
   end                 
  endcase
 
 end
 
 endmodule
 
 //***************************** ALU Control Unit *********************************************************
 module alucontrol(output reg [3:0] alu_op,
                   input  [5:0] fun,
                   input  [1:0] ALUop,
                   input clk); 
                   
                always @(posedge clk)
                 begin
                  if(ALUop == 2'b00)  // load and store word
                    begin 
                      alu_op = 4'b0000;
                    end
                  else if(ALUop == 2'b01)  // branch equal
                    begin
                      alu_op = 4'b0001;
                    end
                       
                  else if(ALUop == 2'b10)  // R type
                    begin
                      if(fun == 6'b100000) 
                         alu_op = 4'b0000;
                      else if(fun == 6'b100001)
                         alu_op = 4'b0001;
                      else if(fun == 6'b100010)
                         alu_op = 4'b0010; 
                      else if(fun == 6'b100011)
                          alu_op = 4'b0011;
                      else if(fun == 6'b100100)
                           alu_op = 4'b0100;
                      else if(fun == 6'b100101)
                           alu_op = 4'b0101;   
                      else if(fun == 6'b100110)
                           alu_op = 4'b0110;
                      else if(fun == 6'b100111)
                           alu_op = 4'b0111;
                      else if(fun == 6'b101000)
                            alu_op = 4'b1000;
                    end
                   end
endmodule


//**************************************** EX/MEM Latch ***********************************************************
module Latch2(output reg [31:0] BranchAddress, ex_memAluOut, ex_memReadData2,
              output reg zeroOut,
              output reg [1:0] ex_memWB, 
              output reg [4:0]ex_memMuxout,
              output reg branch, memWrite, memRead, 
              input [1:0] id_exWB, 
              input [2:0] id_exM, 
              input [31:0] addOut, alu_out, 
              input zero, 
              input [4:0] muxout, 
              input [31:0]id_exReadData2, 
              input clk );
              
              always @(posedge clk)
                begin 
                  BranchAddress <= addOut;
                  ex_memAluOut <= alu_out;
                  ex_memReadData2 <= id_exReadData2;
                  zeroOut <= zero;
                  ex_memWB <= id_exWB;
                  ex_memMuxout <= muxout;
                  branch  <=  id_exM[0];
                  memWrite <= id_exM[2];
                  memRead  <= id_exM[1];
                end
endmodule


//**************************************** Memory Access Stage ****************************************************
module MEM(output [31:0] data_out,
           output PCSrc,
           input branch, memWrite, memRead, zeroOut,
           input [31:0] ex_memAluOut, ex_memReadData2,
           input clk);
           
           and G1(PCSrc, branch, zeroOut);
           datamemory DataMemory(data_out, ex_memReadData2, ex_memAluOut, memRead, memWrite, clk);
endmodule


//*************************************** MEM/WB Latch ************************************************************
module Latch3(output reg memtoReg, regWrite,
               output reg [4:0] writeAddress,
               output reg [31:0] mem_wbDataOut, mem_wbAluOut,
               input [4:0] ex_memMuxout, 
               input [31:0] ex_memAluOut, data_out,
               input [1:0] ex_memWB, 
               input clk);
               
               always @(posedge clk)
                 begin
                   memtoReg <= ex_memWB[1];
                   regWrite <= ex_memWB[0];
                   writeAddress <= ex_memMuxout;
                   mem_wbDataOut <= data_out;
                   mem_wbAluOut  <= ex_memAluOut;
                 end
endmodule


//**************************************** Data Memory ************************************************************
module datamemory(data_out, data_in, addr, mr, mw, clk);
parameter addr_size = 32, word_size = 32,
         memory_size = 1024;
input [addr_size-1:0] addr;
input [word_size-1:0] data_in;
input  mr, mw, clk;
output [word_size-1:0] data_out;
reg [word_size-1:0] mem [0:memory_size-1];
assign data_out = (mr)?mem[addr]:8'hz;
always @(posedge clk)
  begin
    if (mw) mem[addr] = data_in;
  end
endmodule


//***************************** Write Back (WB) ******************************************************************
module Wb(output [31:0] muxout1,
          input memtoReg,
          input [31:0] mem_wbDataOut, mem_wbAluOut);
          
          mux MUX5(muxout1, mem_wbAluOut, mem_wbDataOut, memtoReg);

endmodule


//***************************** Main Control Unit *****************************************************************
module control_path(output reg [1:0] WB,
                    output  reg [2:0] M,
                    output  reg [3:0] EX,
                    output  reg jump,
                    input [5:0] a,
                    input clk);
                    
                    always @(posedge clk)
                     begin
                         if(a == 6'b000000)   // R-type instruction
                           begin
                            jump  <= 1'b0;
                            WB    <= 2'b01;
                            M     <= 3'b0001;
                            EX    <= 4'b1001;
                            
                           // RegDst <= 1'b1;
                           // branch <= 1'b0; 
                          //  memRead <= 1'b0; 
                            
                          //  memtoReg <= 1'b0; 
                          //  memWrite  <= 1'b0; 
                            
                          //  aluSrc   <= 1'b0;
                          //  regWrite <= 1'b1;
                         //   ALUop   <= 2'b10;
                          end
                        else if(a == 6'b000001)      // Load Word
                          begin
                            jump <= 1'b0;
                            WB   <= 2'b11;
                            M    <= 3'b010;
                            EX    <=4'b0010;
                            
                            
                          //  RegDst <= 1'b0;
                          //  branch <= 1'b0; 
                          //  memRead <= 1'b1; 
                          //  memtoReg <= 1'b1; 
                          //  memWrite  <= 1'b0; 
                         //   aluSrc   <= 1'b1;
                         //   regWrite <= 1'b1;
                          // ALUop   <= 2'b00;
                         end
                       else if(a == 6'b000010)     // Store Word
                          begin
                           jump  <= 1'b0;
                           WB    <= 2'b00;
                           M    <=  3'b100;
                           EX   <=  4'b0010;
                           
                           
                        //   RegDst <= 1'b0;
                         //  branch <= 1'b0; 
                        //   memRead <= 1'b0; 
                        //   memtoReg <= 1'b0; 
                        //   memWrite  <= 1'b1; 
                        //   aluSrc   <= 1'b1;
                         //  regWrite <= 1'b0;
                        //  ALUop   <= 2'b00;
                        end
                       else if(a == 6'b000011)     // Branch eq.
                           begin
                            jump <= 1'b0;
                            WB   <= 2'b00;
                            M    <= 3'b001;
                            EX   <=4'b0100;
                            
                           // RegDst <= 1'b0;
                          //  branch <= 1'b1; 
                          //  memRead <= 1'b0; 
                         //   memtoReg <= 1'b0; 
                         //   memWrite  <= 1'b0; 
                         //   aluSrc   <= 1'b0;
                         //   regWrite <= 1'b0;
                          // ALUop   <= 2'b01;
                           end
                       else if(a == 6'b000100)        // Jump
                          begin
                            jump  <= 1'b1;
                            WB    <= 2'b00;
                            M     <= 3'b000;
                            EX    <= 4'b0000;
                           end
                       end                
 endmodule