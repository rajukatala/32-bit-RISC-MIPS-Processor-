`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/07/2021 05:56:15 PM
// Design Name: 
// Module Name: test_bench
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


module test_bench();
 reg clk ,rest;
 wire [5:0] a;
 wire [1:0] ALUop;
 Data_Path     DP(a,ALUop,clk,rest, RegDst, branch, memRead, memtoReg,memWrite, aluSrc, regWrite);
 control_path  CP(RegDst, branch, memRead, memtoReg, memWrite, aluSrc, regWrite,ALUop,a,clk,rest);
 initial 
 begin 
 clk = 1'b0; 
 //#3start = 1'b1;
 //#8 clr = 1'b1; 
 rest = 1'b0;
 #50 rest = 1'b1;
 #1000 $finish; 
 end 
 always #5 clk = ~clk;
endmodule
