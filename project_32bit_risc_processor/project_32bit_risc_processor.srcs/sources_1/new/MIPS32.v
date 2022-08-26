`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// College: IIIT ALLAHABAD 
// Engineer: Rajuram katala
// 
// Create Date: 09/05/2021 08:02:35 PM
// Design Name: 32 bit Risc Processor
// Module Name: control_path
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

module control_path(output reg RegDst, branch, memRead, memtoReg, memWrite, aluSrc, regWrite,
                    output reg [1:0]ALUop,
                    input [5:0] a,
                    input clk,rest);
                    always @(posedge clk or negedge rest)
                      begin
                        if(!rest)
                          begin
                            RegDst <= 1'b0;
                            branch <= 1'b0; 
                            memRead <= 1'b0; 
                            memtoReg <= 1'b0; 
                            memWrite  <= 1'b0; 
                            aluSrc   <= 1'b0;
                            regWrite <= 1'b0;
                            ALUop   <= 2'b00;
                          end
                        else if(a == 6'b000000)   // R-type instruction
                           begin
                            RegDst <= 1'b1;
                            branch <= 1'b0; 
                            memRead <= 1'b0; 
                            memtoReg <= 1'b0; 
                            memWrite  <= 1'b0; 
                            aluSrc   <= 1'b0;
                            regWrite <= 1'b1;
                            ALUop   <= 2'b10;
                          end
                        else if(a == 6'b000001)      // Load Word
                          begin
                            RegDst <= 1'b0;
                            branch <= 1'b0; 
                            memRead <= 1'b1; 
                            memtoReg <= 1'b1; 
                            memWrite  <= 1'b0; 
                            aluSrc   <= 1'b1;
                            regWrite <= 1'b1;
                           ALUop   <= 2'b00;
                         end
                       else if(a == 6'b000010)     // Store Word
                          begin
                           RegDst <= 1'b0;
                           branch <= 1'b0; 
                           memRead <= 1'b0; 
                           memtoReg <= 1'b0; 
                           memWrite  <= 1'b1; 
                           aluSrc   <= 1'b1;
                           regWrite <= 1'b0;
                          ALUop   <= 2'b00;
                        end
                       else if(a == 6'b000011)     // Beq
                           begin
                            RegDst <= 1'b0;
                            branch <= 1'b1; 
                            memRead <= 1'b0; 
                            memtoReg <= 1'b0; 
                            memWrite  <= 1'b0; 
                            aluSrc   <= 1'b0;
                            regWrite <= 1'b0;
                           ALUop   <= 2'b01;
                         end
                       end     
                        
endmodule

module Data_Path(output [5:0] a,
                    input [1:0] ALUop,
                    input clk,rest, RegDst, branch, memRead, memtoReg,
                          memWrite, aluSrc, regWrite
                  );
             wire [31:0] instruction, addOut, snx, readdata1, readdata2, writedata,Branch; 
            IF        IF_STAGE(instruction, addOut, pcsrc, clk, rest, Branch);
            ID        ID_STAGE(readdata1, readdata2, snx, a, instruction, writedata, RegDst, regWrite, clk, rest);
            EX_MEM_WB  EX_MEM_WB_STAGE(writedata, Branch, pcsrc, readdata1, readdata2, snx, addOut, branch, aluSrc, ALUop, memRead,
                                           memtoReg, memWrite, clk, rest);                    
endmodule

module IF(output [31:0] instruction, addOut,
          input pcsrc,clk,rest,
          input [31:0] Branch
          );
          wire [31:0] muxout,pcout;
        mux MUX1(muxout, addOut, Branch, pcsrc);
        pc   PC(pcout, muxout,clk,rest);
        add  Adder(addOut, pcout);
        IM   Instection_Memory(instruction, pcout);
endmodule
module mux(output [31:0] muxout,
           input [31:0] addOut, Branch,
           input pcsrc );
           
           assign muxout = pcsrc?Branch:addOut;

endmodule
module pc(output reg [31:0] pcout,
          input [31:0] muxout,
          input clk,rest);
          always @(posedge clk or negedge rest)
             if(!rest)
                begin
                  pcout = 8'h0;
                end
              else
                begin
                  pcout <= muxout;
                end
endmodule
module add(output [31:0] addOut,
           input [31:0] pcout
           );
assign addOut = pcout + 8'h00000004;
endmodule
module IM(output reg [31:0] instruction,
          input [31:0] pcout);
          wire [31:0]mem[0:31];
 assign mem[0] = 32'b0000_0000_0100_0010_0001_1000_0010_0000; // fn1 R type
 assign mem[4] = 32'b0000_0000_0100_0010_0001_1000_0010_0001; // fn2 rR typr
 assign mem[8] = 32'b0000_0100_0100_0010_0001_1000_0010_0010; // fn3 load word
 assign mem[12] = 32'b0000_0000_0100_0010_0001_1000_0010_0011; // fn4 R type
 assign mem[16] = 32'b0000_1000_0100_0010_0001_1000_0010_0100; // fn5 store word
 assign mem[20] = 32'b0000_0000_0100_0010_0001_1000_0010_0101; // fn6 R type
 assign mem[24] = 32'b0000_0000_0100_0010_0001_1000_0010_0110; // fn7 R type
 assign mem[28] = 32'b0000_1100_0100_0010_0001_1000_0010_0111; // fn8 branch eq
 assign mem[1]  = 32'b0000_0000_0100_0010_0001_1000_0010_0111;
 assign mem[2]  = 32'b0000_0000_0100_0010_0001_1000_0010_0111;
 assign mem[3]  = 32'b0000_0000_0100_0010_0001_1000_0010_0111;
 assign mem[5]  = 32'b0000_0000_0100_0010_0001_1000_0010_0111;
 assign mem[6]  = 32'b0000_0000_0100_0010_0001_1000_0010_0111;
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
module ID(output [31:0] readdata1, readdata2, snx,
          output [5:0] a,
          input [31:0] instruction, writedata,
          input RegDst, regWrite, clk, rest
          );
          wire [4:0] rr1,rr2,wr,muxout;
          wire [15:0] se;
          buf    B1(a[5],instruction[31]),
                 B11(a[4],instruction[30]),
                 B12(a[3],instruction[29]),
                 B13(a[2],instruction[28]),
                 B14(a[1],instruction[27]),
                 B15(a[0],instruction[26]),
                    B21(rr1[4],instruction[25]),
                    B22(rr1[3],instruction[24]),
                    B23(rr1[2],instruction[23]),
                    B24(rr1[1],instruction[22]),
                    B25(rr1[0],instruction[21]),
                    B31(rr2[4],instruction[20]),
                    B32(rr2[3],instruction[19]),
                    B33(rr2[2],instruction[18]),
                    B34(rr2[1],instruction[17]),
                    B35(rr2[0],instruction[16]),
                    B41(wr[4],instruction[15]),
                    B42(wr[3],instruction[14]),
                    B43(wr[2],instruction[13]),
                    B44(wr[1],instruction[12]),
                    B45(wr[0],instruction[1]),
                    B51(se[15],instruction[15]),
                    B52(se[14],instruction[14]),
                    B53(se[13],instruction[13]),
                    B54(se[12],instruction[12]),
                     B55(se[11],instruction[11]),
                     B56(se[10],instruction[10]),
                     B61(se[9],instruction[9]),
                     B62(se[8],instruction[8]),
                     B63(se[7],instruction[7]),
                     B64(se[6],instruction[6]),
                     B66(se[5],instruction[5]),
                     B65(se[4],instruction[4]),
                     B67(se[3],instruction[3]),
                     B69(se[2],instruction[2]),
                     B68(se[1],instruction[1]),
                     B(se[0],instruction[0]);                                                   
          mmux    MUX2(muxout, rr2, wr, RegDst);
          regbank REGBENK(readdata1, readdata2, rr1, rr2, writedata, muxout, regWrite, clk, rest);
          signex  SIGN_EXTEND(snx,se);
          
endmodule
module mmux(output [4:0] muxout,
           input [4:0] rr2, wr,
           input RegDst );
           
           assign muxout = RegDst?wr:rr2;

endmodule
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
module signex(output [31:0] snx,
              input [15:0] se);
               
             assign snx = {4'h0000,se};
endmodule

module EX_MEM_WB(output [31:0] writedata, Branch,
                 output pcsrc,
                 input [31:0] readdata1, readdata2, snx, addOut,
                 input branch, aluSrc,
                 input [1:0] ALUop,
                 input memRead, memtoReg, memWrite, clk, rest
                );
                 wire [31:0] sl,muxout,readdata,alu_out;
                 wire [3:0] alu_op;
                 wire [5:0] fun;
                 buf  a(fun[5],snx[5]),
                      a2(fun[4],snx[4]),
                      a3(fun[3],snx[3]),
                      a4(fun[2],snx[2]),
                      a44(fun[1],snx[1]),
                      a6(fun[0],snx[0]);
                 assign     sl = snx<<2;
                 mux        MUX3(muxout, readout2, snx, aluSrc);
                 adder      ADDER(Branch, addOut, sl);
                 ALU        ALU(alu_out, status, readdata1, muxout, alu_op);
                 alucontrol ALU_CONTROL(alu_op, fun, ALUop, clk);
                and         G1(pcsrc, branch, status);
                datamemory  DATA_MEMORY(readdata, readdata2, alu_out, memRead, memWrite, clk);
                mux         MUX4(writedata, alu_out, readdata, memtoReg);
endmodule
module adder(output [31:0] Branch,
             input [31:0] addOut,sl);
             
             assign Branch = sl + addOut;
             
 endmodule
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
 
  ADD: begin                      // Use carry look Ahead Adder(code in separate file)
 
    alu_out = a + b;
    status  = (alu_out == 32'b0);
 
   end
 
  SUB: begin                     // Add using 2' commpiment
 
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
  MUL: begin                    // Use Booth Algorithms (code in separate Algorithms)
 
    alu_out = a* b;
 
    status  = (alu_out == 32'b0);
 
   end
  DIV:  begin               // Use NOn-Restoring Algoritms(code In separate file)
   
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
module RISC_PROCESSOR(input clk, rest);
 wire RegDst, branch, memRead, memtoReg, memWrite, aluSrc, regWrite;
 wire [1:0] ALUop;
 wire [5:0] a;
  Data_Path(a,ALUop,clk,rest, RegDst, branch, memRead, memtoReg,memWrite, aluSrc, regWrite);
  control_path(RegDst, branch, memRead, memtoReg, memWrite, aluSrc, regWrite,ALUop,a,clk,rest);
endmodule               