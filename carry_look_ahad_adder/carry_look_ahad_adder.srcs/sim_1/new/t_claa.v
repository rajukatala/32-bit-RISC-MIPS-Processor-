`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/11/2021 11:16:20 AM
// Design Name: 
// Module Name: t_claa
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


module t_claa();
reg [3:0] a,b;
reg c0;
wire [3:0] d;
wire c;
claa m(.a(a),.b(b),.c0(c0),.d(d),.c(c));
initial
begin
c0=1'b1;
a[3:0]=4'b0011;
b[3:0]=4'b1100;
#10
c0=1'b0;
a[3:0]=4'b0001;
b[3:0]=4'b1100;
#10
c0=1'b0;
a[3:0]=4'b0011;
b[3:0]=4'b1110;
#10
c0=1'b0;
a[3:0]=4'b0011;
b[3:0]=4'b1111;
#10
c0=1'b0;
a[3:0]=4'b0000;
b[3:0]=4'b1110;
#10
c0=1'b0;
a[3:0]=4'b0001;
b[3:0]=4'b1000;
#10
$finish;
end
endmodule
