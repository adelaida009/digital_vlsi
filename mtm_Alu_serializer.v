module mtm_Alu_serializer(
  input  wire clk,   // posedge active clock
  input  wire rst_n,
  input wire [31:0] C,
  input wire [7:0] CTL_out,
  output reg sout
  );

  localparam IDLE = 4'd0;  //-- IDLE reposo
  localparam SERIAL_C = 4'd1;
  localparam SERIAL_CTL = 4'd2;
  localparam BEGIN = 4'd3; //-- Receiving data
  localparam BEGIN_2 = 4'd4;  //-- Storing the character received
  localparam FINISH = 4'd5;   //-- Data is available
  localparam NEXT = 4'd6;

  reg [7:0] bit_counter;
  reg [7:0] byte_counter;
  reg [2:0] state;
  reg [31:0] C_nxt;
  reg [7:0] CTL_nxt;

  initial begin
    state = IDLE;
    bit_counter = 0;
    byte_counter = 0;
  end

  always @(posedge clk) begin
    if (!rst_n) begin
      state = IDLE;
      bit_counter = 0;
      byte_counter = 0;
      CTL_nxt = 0;
      C_nxt = 0;
      sout = 1;
    end
    else begin
      case(state)
        IDLE: begin
          if (CTL_out[7] == 0) begin
            state = BEGIN;
            C_nxt = C;
            CTL_nxt = CTL_out;
            byte_counter = 5;
            bit_counter = 31;
          end
          else if (CTL_out[7:0] == 8'b11001001 || CTL_out[7:0] == 8'b10010011 || CTL_out[7:0] == 8'b10100101) begin
            state = BEGIN;
            C_nxt = C;
            CTL_nxt = CTL_out;
            byte_counter = 1;
          end
          else begin
            sout = 1;
          end
        end
        BEGIN: begin
          state = BEGIN_2;
          sout = 0;
          end
        BEGIN_2: begin
          if (byte_counter > 1) begin
            sout = 0;
            state = SERIAL_C;
          end
          else begin
            sout = 1;
            state = SERIAL_CTL;
            bit_counter = 7;
          end
        end
        SERIAL_C: begin
          if ((bit_counter+1)% 8 == 1) begin
            sout = C_nxt[bit_counter];
            state = FINISH;
            byte_counter = byte_counter - 1;
            bit_counter = bit_counter - 1;
          end
          else begin
            sout = C_nxt[bit_counter];
            bit_counter = bit_counter - 1;
          end
        end
        SERIAL_CTL: begin
          if (bit_counter == 0) begin
            sout = CTL_nxt[bit_counter];
            state = FINISH;
            byte_counter = byte_counter - 1;
          end
          else begin
            sout = CTL_nxt[bit_counter];
            bit_counter = bit_counter - 1;
          end
        end
        FINISH: begin
          sout = 1;
          if (byte_counter == 0) begin
            state = IDLE;
            end
            else
          state = BEGIN;
          end
      endcase
    end
  end
    endmodule
