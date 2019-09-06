/******************************************************************************
 * (C) Copyright 2019 AGH UST All Rights Reserved
 *
 * MODULE:    mtm_Alu tb
 * PROJECT:   PPCU_VLSI
 * AUTHORS:
 * DATE:
 * ------------------------------------------------------------------------------
 * This module (TB) provides test patterns for the ALU, reads data from the ALU and
 * verifies if the operation result is correct.
 *
 * The TB must include:
 * - task send_byte to send a CMD or CTL command to the ALU
 * - task send_calculation_data that will send 9 bytes to the ALU for given
 *   operands and operation
 * - procedural block for capturing the input data from the ALU
 * - task compare to compare the result from the ALU and the expected data.
 *
 * The test vectors must provide at least:
 * - sending max (0xFFFF) and min (0) data with all the ALU operations (AND OR, ADD,SUB)
 * - sending 1000 random valid data
 * - sending invalid data (wrong number of DATA packets before CTL packet)
 * - sending data with CRC error
 *
 * The testbench should print final PASS/FAIL text information.
 */

module mtm_Alu_tb (
    output reg clk,
    output reg rst_n,
    output reg sin,
    input wire sout
) ;

integer i,j,k,l,j_nxt,l_nxt,g ;

reg [54:0] out, out_nxt;
reg [31:0] A, B;
reg [7:0] CTL;
reg pass, passes;
reg [67:0] data;
reg [2:0] OP;

initial begin
  sin = 1; clk = 0; i = 0; j = 0; k = 0; l = 0; passes = 1; g = 0;
  rst_n = 0;
  out=54'b111111111111111111111111111111111111111111111111111111;
  out_nxt=54'b111111111111111111111111111111111111111111111111111111;
  #400 rst_n = 1;

  #400 $display("Send 1000 random valid data");
  while (g < 1000)
    begin
      if (g < 250)
        OP = 3'b000;
      else if (500 < g > 250)
        OP = 3'b100;
      else if (750 < g > 500)
        OP = 3'b001;
      else if (1000 < g > 750)
        OP = 3'b101;
      #1000 g = g + 1;
      valid_data(OP,pass);
      passes = passes & pass;
    end
  if (passes)
    $display("PASSED");
  else
    $display("FAILED");

  #1000$display("Corner cases");
  #1000corner_cases(pass);
  if (pass)
    $display("PASSED");
  else
    $display("FAILED");

  $display("Send valid data with crc error");
  A = 5;
  B = 2;
  CTL = 8'b01000000;
  send_calculation_data({CTL,B,A});
  #15000 pass = {out[52:45]} == 8'b10100101;
  if (pass)
    $display("PASSED");
  else
  $display("FAILED");

  send_byte(8'b01010000,0);
  send_byte(8'b01010000,1);
  #10000 $display("sending invalid data (wrong number of DATA packets before CTL packet)");
  if (out[52:45] == 8'b11001001) begin
    $display("PASSED");
  end
  else
    $display("FAILED");

  $display("sending wrong OP code");
  A = 10 ;
  B = 10;
  CTL = 8'b00101010;
  send_calculation_data({CTL,B,A});
  #10000 $display("waiting for results");
  if (out[52:45] == 8'b10010011) begin
    $display("PASSED");
  end
  else
    $display("FAILED");
  $finish;
end

always @(*) begin
if (l > 0 && l < 55) begin
  out_nxt[l-1] = sout;
  l_nxt = l - 1;
end
else if (l == 0) begin
  l_nxt = 56;
  end
else if (sout == 0) begin
  l_nxt = 54;
  out_nxt[54] = 0;
end
j_nxt = j + 1;
end

always @(posedge clk) begin
j = j_nxt;
out = out_nxt;
l = l_nxt;
end

always
  #100  clk =  ! clk;

task send_byte;
input [7:0] byte;
input [1:0] c; // c=0 for DATA c=1 for CTL
begin
for(i=11;i>0;i=i-1) begin
@(posedge clk); begin
if (c == 0) begin
  if (i == 11 || i == 10)
    #2sin = 0;
  else if (i == 1)
    #2sin = 1;
  else
    #2sin = byte[i-2];
  end
if (c == 1) begin
  if (i == 11 )
    #2sin = 0;
  else if (i == 1 || i == 10)
    #2sin = 1;
  else
    #2sin = byte[i-2];
  end
  end
end
end
endtask

task send_calculation_data;
  input [71:0] byte; //
  begin
  for(k=0;k<9;k=k+1) begin
    @(posedge clk); begin
    if (k==0) begin
      send_byte(byte[31:24],0);
    end
    else if (k==1) begin
      send_byte(byte[23:16],0);
    end
    else if (k==2) begin
      send_byte(byte[15:8],0);
    end
    else if (k==3) begin
      send_byte(byte[7:0],0);
    end
    else if (k==4) begin
      send_byte(byte[63:56],0);
    end
    else if (k==5) begin
      send_byte(byte[55:48],0);
    end
    else if (k==6) begin
      send_byte(byte[47:40],0);
    end
    else if (k==7) begin
      send_byte(byte[39:32],0);
    end
    else if (k==8) begin
      send_byte(byte[71:64],1);
    end
    end
  end
end
endtask


function [3:0] nextCRC4_D68;
// polynomial: x^4 + x^1 + 1
// data width: 68
// convention: the first serial bit is D[67]

  input [67:0] Data;
  input [3:0] crc;
  reg [67:0] d;
  reg [3:0] c;
  reg [3:0] newcrc;
  begin
    d = Data;
    c = crc;

    newcrc[0] = d[66] ^ d[64] ^ d[63] ^ d[60] ^ d[56] ^ d[55] ^ d[54] ^ d[53] ^ d[51] ^ d[49] ^ d[48] ^ d[45] ^ d[41] ^ d[40] ^ d[39] ^ d[38] ^ d[36] ^ d[34] ^ d[33] ^ d[30] ^ d[26] ^ d[25] ^ d[24] ^ d[23] ^ d[21] ^ d[19] ^ d[18] ^ d[15] ^ d[11] ^ d[10] ^ d[9] ^ d[8] ^ d[6] ^ d[4] ^ d[3] ^ d[0] ^ c[0] ^ c[2];
    newcrc[1] = d[67] ^ d[66] ^ d[65] ^ d[63] ^ d[61] ^ d[60] ^ d[57] ^ d[53] ^ d[52] ^ d[51] ^ d[50] ^ d[48] ^ d[46] ^ d[45] ^ d[42] ^ d[38] ^ d[37] ^ d[36] ^ d[35] ^ d[33] ^ d[31] ^ d[30] ^ d[27] ^ d[23] ^ d[22] ^ d[21] ^ d[20] ^ d[18] ^ d[16] ^ d[15] ^ d[12] ^ d[8] ^ d[7] ^ d[6] ^ d[5] ^ d[3] ^ d[1] ^ d[0] ^ c[1] ^ c[2] ^ c[3];
    newcrc[2] = d[67] ^ d[66] ^ d[64] ^ d[62] ^ d[61] ^ d[58] ^ d[54] ^ d[53] ^ d[52] ^ d[51] ^ d[49] ^ d[47] ^ d[46] ^ d[43] ^ d[39] ^ d[38] ^ d[37] ^ d[36] ^ d[34] ^ d[32] ^ d[31] ^ d[28] ^ d[24] ^ d[23] ^ d[22] ^ d[21] ^ d[19] ^ d[17] ^ d[16] ^ d[13] ^ d[9] ^ d[8] ^ d[7] ^ d[6] ^ d[4] ^ d[2] ^ d[1] ^ c[0] ^ c[2] ^ c[3];
    newcrc[3] = d[67] ^ d[65] ^ d[63] ^ d[62] ^ d[59] ^ d[55] ^ d[54] ^ d[53] ^ d[52] ^ d[50] ^ d[48] ^ d[47] ^ d[44] ^ d[40] ^ d[39] ^ d[38] ^ d[37] ^ d[35] ^ d[33] ^ d[32] ^ d[29] ^ d[25] ^ d[24] ^ d[23] ^ d[22] ^ d[20] ^ d[18] ^ d[17] ^ d[14] ^ d[10] ^ d[9] ^ d[8] ^ d[7] ^ d[5] ^ d[3] ^ d[2] ^ c[1] ^ c[3];
    nextCRC4_D68 = newcrc;
end
endfunction

task compare;
  input [31:0] result;
  input [31:0] A;
  input [31:0] B;
  input [7:0] CTL;
  reg [31:0] C;
  output ret;
  begin
    case(CTL[6:4])
    3'b000: begin
      C = A & B;
    end
    3'b001: begin
      C = A | B;
    end
    3'b100: begin
      C = A + B;
    end
    3'b101: begin
      C = A - B;
    end
    endcase
  ret = C == result;
  end
endtask


task valid_data;
  input [2:0] OP;
  reg [31:0] A;
  reg [31:0] B;
  reg [7:0] CTL;
  reg [67:0] data;
  reg [39:0] result;
  output pass;
  begin
    A = $urandom;
    B = $urandom;
    CTL = {1'b0,OP,4'b0000};
    data = {A,B,1'b1,OP};
    CTL[3:0] = nextCRC4_D68(data,4'b0000);
    send_calculation_data({CTL,B,A});
    #20000 compare({out[52:45],out[41:34],out[30:23],out[19:12]},A,B,CTL,pass);
  end
endtask

task corner_cases;
  reg [31:0] A;
  reg [31:0] B;
  reg [7:0] CTL;
  reg [67:0] data;
  reg [39:0] result;
  reg pass;
  output passes;
  begin
    passes = 1;
    A=32'b11111111111111111111111111111111;
    B=32'b11111111111111111111111111111111;
    CTL = 8'b00000000;
    data = {A,B,1'b1,CTL[6:4]};
    CTL[3:0] = nextCRC4_D68(data,4'b0000);
    send_calculation_data({CTL,B,A});
    #20000 compare({out[52:45],out[41:34],out[30:23],out[19:12]},A,B,CTL,pass);
    passes = passes & pass;
    CTL = 8'b00010000;
    data = {A,B,1'b1,CTL[6:4]};
    CTL[3:0] = nextCRC4_D68(data,4'b0000);
    send_calculation_data({CTL,B,A});
    #20000 compare({out[52:45],out[41:34],out[30:23],out[19:12]},A,B,CTL,pass);
    passes = passes & pass;
    CTL = 8'b01000000;
    data = {A,B,1'b1,CTL[6:4]};
    CTL[3:0] = nextCRC4_D68(data,4'b0000);
    send_calculation_data({CTL,B,A});
    #20000 compare({out[52:45],out[41:34],out[30:23],out[19:12]},A,B,CTL,pass);
    passes = passes & pass;
    CTL = 8'b01010000;
    data = {A,B,1'b1,CTL[6:4]};
    CTL[3:0] = nextCRC4_D68(data,4'b0000);
    send_calculation_data({CTL,B,A});
    #20000 compare({out[52:45],out[41:34],out[30:23],out[19:12]},A,B,CTL,pass);
    passes = passes & pass;
    A=32'b00000000000000000000000000000000;
    B=32'b00000000000000000000000000000000;
    CTL = 8'b00000000;
    data = {A,B,1'b1,CTL[6:4]};
    CTL[3:0] = nextCRC4_D68(data,4'b0000);
    send_calculation_data({CTL,B,A});
    #20000 compare({out[52:45],out[41:34],out[30:23],out[19:12]},A,B,CTL,pass);
    passes = passes & pass;
    CTL = 8'b00010000;
    data = {A,B,1'b1,CTL[6:4]};
    CTL[3:0] = nextCRC4_D68(data,4'b0000);
    send_calculation_data({CTL,B,A});
    #20000 compare({out[52:45],out[41:34],out[30:23],out[19:12]},A,B,CTL,pass);
    passes = passes & pass;
    CTL = 8'b01000000;
    data = {A,B,1'b1,CTL[6:4]};
    CTL[3:0] = nextCRC4_D68(data,4'b0000);
    send_calculation_data({CTL,B,A});
    #20000 compare({out[52:45],out[41:34],out[30:23],out[19:12]},A,B,CTL,pass);
    passes = passes & pass;
    CTL = 8'b01010000;
    data = {A,B,1'b1,CTL[6:4]};
    CTL[3:0] = nextCRC4_D68(data,4'b0000);
    send_calculation_data({CTL,B,A});
    #20000 compare({out[52:45],out[41:34],out[30:23],out[19:12]},A,B,CTL,pass);
    passes = passes & pass;
end
endtask



endmodule
