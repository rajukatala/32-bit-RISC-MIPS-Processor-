`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/24/2022 10:34:51 PM
// Design Name: 
// Module Name: testbench
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


module testbench;
    reg [15:0] Q, M;
    wire [15:0] quo, rem;
    non_restoring_divider inst1(Q,M,quo,rem);
    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0,testbench); //0,1
    end

    initial begin
    $monitor (Q,M,quo,rem);
    Q = 16'd13869;
    M = 16'd900;
    #10
    Q = 16'd901;
    M = 16'd300;
    #10
    Q = 16'd6200;
    M = 16'd1598;
    #10
    Q = 16'd9801;
    M = 16'd310;
    #10
    Q = 16'd15;
    M = 16'd3;
    #10
    Q = 16'd21;
    M = 16'd20;
    #10
    Q = 16'd3000;
    M = 16'd2000;
    #10
    Q = 16'd40000;
    M = 16'd12000;

    end
endmodule
