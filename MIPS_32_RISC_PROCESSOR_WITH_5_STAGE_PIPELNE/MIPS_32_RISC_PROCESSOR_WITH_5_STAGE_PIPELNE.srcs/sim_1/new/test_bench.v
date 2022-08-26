`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/06/2021 08:47:31 PM
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
reg clk , rest;
 wire PCSrc, jump, RegDst, branch, memRead, memtoReg, memWrite, aluSrc, regWrite, zero, zeroOut, ex_memZero;
 MIPS32_PIPELINE MP(clk,rest);
 initial 
  begin 
  clk = 1'b1; 
  //#3start = 1'b1;
  //#8 clr = 1'b1; 
  rest = 1'b0;
  #50 rest = 1'b1;
  #1000 $finish; 
  end 
  always #5 clk = ~clk;

endmodule
