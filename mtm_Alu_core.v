module mtm_Alu_core(
  input wire clk,
  input wire rst_n,
  input wire [31:0] A,
  input wire [31:0] B,
  input wire [7:0] CTL,
  output reg [31:0] C,
  output reg [7:0] CTL_out
  );

wire [2:0] OP;

assign OP = CTL[6:4];
assign CRC = CTL[3:0];

localparam  AND = 3'b000;
localparam OR = 3'b001;
localparam ADD = 3'b100;
localparam SUB = 3'b101;

reg cr,of,zr,neg;

initial begin
  cr = 0; of = 0; zr = 0; neg = 0;
end



always @(posedge clk) begin
  if (!rst_n) begin
    cr = 0; of = 0; zr = 0; neg = 0;
    CTL_out = 8'b11111111;
    C = 0;
  end
  else if (CTL[7] == 0) begin
    #1case(OP)
      AND: begin
        C = A & B;
        CTL_out = {1'b0,cr,of,zr,neg,nextCRC3_D36({C,1'b0,cr,of,zr,neg},4'b0000)};
      end
      OR: begin
        C = A | B;
        CTL_out = {1'b0,cr,of,zr,neg,nextCRC3_D36({C,1'b0,cr,of,zr,neg},4'b0000)};
      end
      ADD: begin
        {cr,C} = A + B;
        zr = (C == 0);
        of = ((A[31] == 0 && B[31] == 0 && C[31] == 1) || (A[31] == 1 && B[31] == 1 && C[31] == 0));
        neg = (C[31] == 1);
        CTL_out = {1'b0,cr,of,zr,neg,nextCRC3_D36({C,1'b0,cr,of,zr,neg},4'b0000)};
      end
      SUB: begin
        {cr,C} = A - B;
        zr = (C == 0);
        of = ((A[31] == 0 && B[31] == 0 && C[31] == 1) || (A[31] == 1 && B[31] == 1 && C[31] == 0));
        neg = (A < B);
        CTL_out = {1'b0,cr,of,zr,neg,nextCRC3_D36({C,1'b0,cr,of,zr,neg},4'b0000)};
      end
      default: begin
        CTL_out = 8'b10010011;
      end
    endcase
    end
    else if( CTL == 8'b10100101 || CTL == 8'b11001001) begin
      CTL_out = CTL;
    end
    else begin
    cr = 0; // ISSUE
    of = 0;
    zr = 0;
    neg = 0;
    CTL_out = 8'b11111111;
    end
  end

  // polynomial: x^3 + x^1 + 1
    // data width: 36
    // convention: the first serial bit is D[35]
function [2:0] nextCRC3_D36;
  input [35:0] Data;
  input [2:0] crc;
  reg [35:0] d;
  reg [2:0] c;
  reg [2:0] newcrc;
begin
  d = Data;
  c = crc;

  newcrc[0] = d[35] ^ d[32] ^ d[31] ^ d[30] ^ d[28] ^ d[25] ^ d[24] ^ d[23] ^ d[21] ^ d[18] ^ d[17] ^ d[16] ^ d[14] ^ d[11] ^ d[10] ^ d[9] ^ d[7] ^ d[4] ^ d[3] ^ d[2] ^ d[0] ^ c[2];
  newcrc[1] = d[35] ^ d[33] ^ d[30] ^ d[29] ^ d[28] ^ d[26] ^ d[23] ^ d[22] ^ d[21] ^ d[19] ^ d[16] ^ d[15] ^ d[14] ^ d[12] ^ d[9] ^ d[8] ^ d[7] ^ d[5] ^ d[2] ^ d[1] ^ d[0] ^ c[0] ^ c[2];
  newcrc[2] = d[34] ^ d[31] ^ d[30] ^ d[29] ^ d[27] ^ d[24] ^ d[23] ^ d[22] ^ d[20] ^ d[17] ^ d[16] ^ d[15] ^ d[13] ^ d[10] ^ d[9] ^ d[8] ^ d[6] ^ d[3] ^ d[2] ^ d[1] ^ c[1];
  nextCRC3_D36 = newcrc;
end
endfunction

endmodule
