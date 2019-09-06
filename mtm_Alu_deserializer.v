module mtm_Alu_deserializer (
  input wire clk,
  input wire rst_n,
  input wire sin,
  output reg [31:0] A,
  output reg [31:0] B,
  output reg [7:0] CTL
);

localparam IDLE = 2'd0;
localparam LOAD = 2'd1;
localparam DATA_ERR = 2'd2;
localparam STOP = 2'd3;

reg [7:0] bit_counter;
reg [1:0] state;
reg [3:0] CRC;
reg [98:0] OUT;

initial begin
  state = IDLE;
  bit_counter = 0;
  CTL = 8'b11111111;
  A = 32'b11111111111111111111111111111111;
  B = 32'b11111111111111111111111111111111;
end

always @(posedge clk) begin
  if (!rst_n) begin
    state = IDLE;
    bit_counter = 0;
    CTL = 8'b11111111;
    OUT = 99'b111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111;
  end
  else begin
    case(state)
      IDLE: begin
        if (sin == 0) begin
          bit_counter = 97;
          state = LOAD;
          OUT = {OUT,sin};
        end
        else begin
          state = IDLE;
        end
      end
      LOAD: begin
        if (bit_counter % 11 == 10 && sin == 1) begin
          state = LOAD;
        end
        else if (bit_counter == 9 && sin == 0) begin
          state = DATA_ERR;
          bit_counter = 9;
          CTL = 8'b11001001;
        end
        else if (bit_counter != 9 && bit_counter % 11 == 9 && sin == 1) begin
          state = DATA_ERR;
          bit_counter = 9;
          CTL = 8'b11001001;
        end
        else if (bit_counter % 11 == 0 && sin == 0) begin
          state = DATA_ERR;
          bit_counter = 0;
          CTL = 8'b11001001;
        end
        else if (bit_counter == 0 && sin == 1) begin
          OUT = {OUT,sin};
          CRC = nextCRC4_D68({OUT[96:89],OUT[85:78],OUT[74:67],OUT[63:56],OUT[52:45],OUT[41:34],OUT[30:23],OUT[19:12],1'b1,OUT[7:5]},4'b0000);
          bit_counter = 2;
          if (CRC == OUT[4:1]) begin
            A = {OUT[96:89],OUT[85:78],OUT[74:67],OUT[63:56]};
            B = {OUT[52:45],OUT[41:34],OUT[30:23],OUT[19:12]};
            CTL = OUT[8:1];
            state = STOP;
          end
          else begin
            CTL = 8'b10100101;
            state = DATA_ERR;
          end
        end
        else begin
          OUT = {OUT,sin};
          bit_counter = bit_counter - 1;
        end
      end
      DATA_ERR: begin
        if (bit_counter > 0) begin
          bit_counter = bit_counter - 1;
        end
        else begin
          state = IDLE;
          CTL = 8'b11111111;
        end
      end
      STOP: begin
        if (bit_counter > 0) begin
          bit_counter = bit_counter - 1;
        end
        else begin
          A = 32'b11111111111111111111111111111111;
          B = 32'b11111111111111111111111111111111;
          CTL = 32'b11111111;
          state = IDLE;
        end
      end
      endcase
    end
  end

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

    endmodule
