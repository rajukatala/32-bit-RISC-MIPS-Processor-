`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/29/2021 12:26:38 PM
// Design Name: 
// Module Name: bsm_test
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

//problab is overflow
module bsm_test();
wire done; 
wire [15:0] z;
reg [7:0] data_in; 
reg clk,start;
datapath DP (q,q1,dc,z,ldm,ldq,lda,rs,ac,t,clk,clra,data_in); 
controlpath CON(ldm,ldq,lda,rs,ac,t,clra,done,q,q1,dc,clk,start); 
//bm DUT(.z(z),.clk(clk),.clr(clr),.a(a),.b(b));
initial 
begin 
clk = 1'b0; 
#3start = 1'b1;
//#8 clr = 1'b1; 
#1000 $finish; 
end 
always #5 clk = ~clk;
initial 
 begin 
 #10data_in = 8'b00000101; 
 #12 data_in =8'b11111010; 
 end 
 initial
 begin 
 $monitor ($time, " %d %b %b %b %b %b %b", z,DP.a,DP.qo,DP.m,DP.q,DP.q1,done); 
$dumpfile ("bsm.vcd"); $dumpvars (0, bsm_test); 
 end 
endmodule