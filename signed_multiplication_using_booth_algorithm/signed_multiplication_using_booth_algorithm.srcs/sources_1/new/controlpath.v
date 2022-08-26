`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/29/2021 11:53:20 AM
// Design Name: 
// Module Name: controlpath
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
module shiftrg ( output reg [7:0] a,
              input [7:0] b,
              input lda,clr,r,clk,g
             );
             always @( posedge clk)
               if(clr)
                 a = 8'b00000000;
               else if(lda)
                  a <=b;
               else if(r)
                  a ={g,a[7:1]};
endmodule
module shiftrg1 ( output reg [7:0] a,
              input [7:0] b,
              input lda,r,clk,g
             );
             always @( posedge clk)
               //if(clr)
                 //a = 4'b0000;
               if(lda)
                  a <=b;
               
               else if(r)
                  a ={g,a[7:1]};
endmodule
module pipo1 ( output reg [7:0] a,
              input [7:0] b,
              input ldq,clk
             );
             always @(posedge clk)
                if(ldq)
                  a <= b;       
endmodule
module alu (output reg [7:0] a,
              input [7:0] b,c,
              input ac,m
              );
              always @(*)
                 if(ac==1 && m==1)
                   a = b - c;
                 else if(ac == 1 && m ==0)
                   a = b+c;
endmodule
//module rshift (output reg [3:0] a,b,
  //             
    //           output reg q,
      //         input [3:0] e,d,
        //       input r,clk
          //     );
            //   reg c;
              // initial begin
//              a = e;
  //             b = d; end
    //           always @( posedge clk)begin
      //         q = d[0];//9783504088
        //         if(r)
          ///         begin
             //       c = d[0];
               //    a = d>>1;
                 //  b = e>>1;
                   // b[3] = c;
                    //end
                 //end   
//endmodule
module counter(output reg [3:0] dec,
               input s,clk,r
               );
              always @(posedge clk)
               if(s)
                dec <= 4'b1000;
               else if(r)
                 dec = dec-4'b0001;
endmodule
module regs(output reg q1,
            input a,clr,clk,r
            );
            always @(posedge clk)
              if(clr)
               q1 = 1'b0;
              else if(r)
                q1 = a;
endmodule               
module datapath( output q,q1,dc,
                 output [15:0] z,
                 input ldm,ldq,lda,rs,ac,t,clk,clra,
                 input [7:0] data_in
                 );
                 wire [7:0] d,a,qo,m;
                 wire [3:0] k;
           //   pipo A (a,d,lda,clk,clra);
            //  pipo1 Q (qo,bus,ldq,clk);
              pipo1 M (m,data_in,ldm,clk);
              alu A1 (d,a,m,ac,t); 
              shiftrg A (a,d,lda,clra,rs,clk,a[3]);
              shiftrg1 Q (qo,data_in,ldq,rs,clk,a[0]);
              counter DC(k,clra,clk,rs);
              regs R (q1,qo[0],clra,clk,rs);
              assign z = {a,qo},
                     q = qo[0],
                     dc = (k==0);
endmodule   
module controlpath( output reg ldm,ldq,lda,rs,ac,t,clra,done,
                    input  q,q1,dc,clk,start
                    );
                    parameter s0 = 3'b000, s1 = 3'b001, s2 = 3'b010, s3 = 3'b011, s4 = 3'b100, s5 = 3'b101;
                    reg [2:0] state;
                    always @(state)
                      begin
                         case(state)
                             s0: begin
                                   ldm =1'b0; 
                                   ldq = 1'b1;
                                   lda = 1'b0;
                                   rs = 1'b0;
                                   ac = 1'b0;
                                   clra = 1'b0;
                                   done =0;
                                  end
                             s1: begin
                                  ldq = 1'b0;
                                   ldm =1'b1;
                                   clra = 1'b1;
                                 end
                             s2: begin
                                   ldm = 1'b0;
                                   clra = 1'b0;
                                   ldq = 0;
                                   ac = 0;
                                  // rs =0;
                                   lda = 0;
                                  if(dc)
                                  begin   done = 1'b1;end
                                 // else if(q)
                                //  begin
                                //     ad = 1'b1;
                                 //    #2 lda = 1'b1;
                               //      #2 rs= 1'b1; end
                              //    else if(!q)
                                   rs = 1'b1;
                                    
                                  end
                             s3: begin
                                  ldm = 1'b0;
                                  clra = 1'b0;
                                  ldq = 0;
                                  rs = 0;
                                  ac = 1;
                                  t = 0;
                                 #2 lda = 1;
                                  
                                  
                                  // lda <= 1'b1;
                                   //r <= 1'b1;
                                   if(dc==1)
                                   begin  done = 1;end
                                 //  else if(q)
                                   //  begin
                                     //  ad = 1;
                                       //#2 lda = 1;
                                       //#2 rs = 1;
                                       //end
                                   //else if(!q)
                                     //  rs = 1;
                                       
                                       
                                 end
                             s4: begin
                                 ldm =0;
                                 clra =0;
                                 rs =0;
                                 ac=1;
                                 t=1;
                                 #2 lda=1;
                                 end
                             s5: begin
                                   ldm =1'b0; 
                                                                                                 ldq = 1'b0;
                                                                                                 lda = 1'b0;
                                                                                                 rs = 1'b0;
                                                                                                 ac = 1'b0;
                                                                                                 clra = 1'b0;
                                                                                                 done =1;
                                                                                                 t=0;
                                                                                                 
                                 end
                            default: begin
                                                                    ldm =1'b0; 
                                                                    ldq = 1'b1;
                                                                    lda = 1'b0;
                                                                    rs = 1'b0;
                                                                    ac = 1'b0;
                                                                    clra = 1'b0;
                                                                    done =0;
                                                                   end
                        endcase
                     end
        always @(posedge clk)
          begin
            case(state)
               s0:begin if(start) state = s1;end
              // s1: state = s2;
               s1: begin
                     if({q,q1}==2'b01)
                       state = s3;
                     else if({q,q1} == 2'b10)
                       state = s4;
                     else
                       state = s2;
                   end
               s2: begin 
                    if(dc)
                       state = s5;
                    else if({q,q1} ==2'b01)
                       state = s3;
                     else if({q,q1} == 2'b10)
                        state = s4;
                    // else if(!q)
                    //     state = s2;
                    end
               s3: begin
                    // if(q)
                     //  state = s2;
                     if(dc)
                       state = s5;
                     else 
                        state = s2;
                   //  else if(!q)
                     //   state = s2;
                    end 
               s4:begin
                     if(dc)
                       state = s5;
                     else
                       state = s2;
                     end
               s5: state =s5;
               default: state = s0;
            endcase
          end               
endmodule       
