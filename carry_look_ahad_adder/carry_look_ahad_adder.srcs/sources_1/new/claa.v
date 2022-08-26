`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// College: IIIT Allahabad 
// Engineer: Rajuram katala
// 
// Create Date: 07/11/2021 10:08:50 AM
// Design Name: 
// Module Name: claa
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
module ha(output p,g,
          input a,b
          );
          and G1 (g,a,b);
          xor G3 (p,a,b);
endmodule
/*module fa(output s,c
          input a,b,c0
          );
          wire p1,g1,g;
          ha HA1(p1,g1,a,b),
             HA2 (s,g,p1,co);
         or G1 (c,g,g1);
endmodule */
module claa(
    output [3:0] d,
    output c,
    input [3:0] a,b,
    input c0
    );
    wire c1,c2,c3,g0,p0,g1,p1,g2,p2,g3,p3;
    ha HA1 (p0,g0,a[0],b[0]),
       HA2 (p1,g1,a[1],b[1]),
       HA3 (p2,g2,a[2],b[2]),
       HA4 (p3,g3,a[3],b[3]);
    assign c1 = g0 |( p0 & c0),
           c2 = g1 | (p1&g0) | (p1&p0&c0),
           c3 = g2 | (p2 & g1) | (p1 & p2 & g0) |( p2 & p1 & p0 & c0),
           c = g3 | (p3 & g2) | (p3 & p2 & g1) | (p3 & p2 & p1 & g0) |(p3 & p2 & p1 & p0 & c0);
           xor G1 (d[0],c0,p0),
               G2 (d[1],c1,p1),
               G3 (d[2],c2,p2),
               G4 (d[3],c3,p3);
     
endmodule
