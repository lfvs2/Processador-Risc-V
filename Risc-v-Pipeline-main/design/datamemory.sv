`timescale 1ns / 1ps

module datamemory #(
    parameter DM_ADDRESS = 9,
    parameter DATA_W = 32
) (
    input logic clk,
    input logic MemRead,  // comes from control unit
    input logic MemWrite,  // Comes from control unit
    input logic [DM_ADDRESS - 1:0] a,  // Read / Write address - 9 LSB bits of the ALU output
    input logic [DATA_W - 1:0] wd,  // Write Data
    input logic [2:0] Funct3,  // bits 12 to 14 of the instruction
    output logic [DATA_W - 1:0] rd  // Read Data
);

  logic [31:0] raddress;
  logic [31:0] waddress;
  logic [31:0] Datain;
  logic [31:0] Dataout;
  logic [ 3:0] Wr;

  Memoria32Data mem32 (
      .raddress(raddress),
      .waddress(waddress),
      .Clk(~clk),
      .Datain(Datain),
      .Dataout(Dataout),
      .Wr(Wr)
  );

  always_comb begin
    raddress = {{22{1'b0}}, a[8:2], 2'b00};
    waddress = {{22{1'b0}}, a[8:2], 2'b00};
    Datain = wd;
    Wr = 4'b0000;
    rd = 32'b0;
    if (MemRead) begin
      case (Funct3)
        3'b000:	begin  //LB
        case (a[1:0])
           	2'b00: rd = {{24{Dataout[7]}}, Dataout[7:0]};   // Load byte 0 (bits 7:0 of Dataout)
           	2'b01: rd = {{24{Dataout[15]}}, Dataout[15:8]}; // Load byte 1 (bits 15:8 of Dataout)
           	2'b10: rd = {{24{Dataout[23]}}, Dataout[23:16]}; // Load byte 2 (bits 23:16 of Dataout)
           	2'b11: rd = {{24{Dataout[31]}}, Dataout[31:24]}; // Load byte 3 (bits 31:24 of Dataout)
        endcase
	end
	3'b100:	begin  //LBU
	case (a[1:0]) 
		2'b00:	rd = {24'b0, Dataout[7:0]};  // Estende com 24 zeros à esquerda
		2'b01:	rd = {24'b0, Dataout[15:8]}; // Estende com 24 zeros à esquerda
		2'b10:	rd = {24'b0, Dataout[23:16]}; // Estende com 24 zeros à esquerda
		2'b11:	rd = {24'b0, Dataout[31:24]}; // Estende com 24 zeros à esquerda
	default:rd = 32'bX; // ou um valor de erro
	endcase
	end
        3'b001:	begin  //LH
	 case (a[1])
              	1'b0: rd = {{16{Dataout[15]}}, Dataout[15:0]}; // Load half-word 0 (bits 15:0 of Dataout)
                1'b1: rd = {{16{Dataout[31]}}, Dataout[31:16]}; // Load half-word 1 (bits 31:16 of Dataout)
                default: rd = 'x; 
        endcase
	end
        3'b010:	begin  //LW
        rd = Dataout;
	end
        default: rd = Dataout;
      endcase
    end else if (MemWrite) begin
      case (Funct3)
      3'b000: begin // SB (Store Byte)
     	case (a[1:0])
      	2'b00: begin Datain = {Dataout[31:8], wd[7:0]}; Wr = 4'b0001; end 
      	2'b01: begin Datain = {Dataout[31:16], wd[7:0], Dataout[7:0]}; Wr = 4'b0010; end 
      	2'b10: begin Datain = {Dataout[31:24], wd[7:0], Dataout[15:0]}; Wr = 4'b0100; end
     	2'b11: begin Datain = {wd[7:0], Dataout[23:0]}; Wr = 4'b1000; end 
      	default: Wr = 4'b0000; 
     	endcase
       end
       3'b001: begin // SH (Store Half-word)
         case (a[1])
         1'b0: begin Datain = {Dataout[31:16], wd[15:0]}; Wr = 4'b0011; end 
         1'b1: begin Datain = {wd[15:0], Dataout[15:0]}; Wr = 4'b1100; end 
         default: Wr = 4'b0000;
         endcase
        end
        3'b010: begin // SW (Store Word)
          Wr = 4'b1111; 
          Datain = wd;  
          end
          default: begin
          Wr = 4'b0000;
          Datain = 'x; 
          end
         endcase
        end
    end

endmodule
 
