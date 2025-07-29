`timescale 1ns / 1ps

module Controller (
    // Inputs
    input logic [6:0] Opcode,
    input logic [2:0] funct3,  // Novo campo para identificar tipo de carregamento

    // Outputs
    output logic ALUSrc,
    output logic MemtoReg,
    output logic RegWrite,
    output logic MemRead,
    output logic MemWrite,
    output logic [1:0] ALUOp,
    output logic Branch,

    // Novos sinais
    output logic [1:0] LoadSize,   // 00: word, 01: half, 10: byte
    output logic       LoadSigned  // 1: signed, 0: unsigned
);

  logic [6:0] R_TYPE, I_TYPE, LW, SW, BR;

  assign R_TYPE = 7'b0110011;
  assign I_TYPE = 7'b0010011;
  assign LW     = 7'b0000011;
  assign SW     = 7'b0100011;
  assign BR     = 7'b1100011;

  assign ALUSrc   = (Opcode == LW || Opcode == SW || Opcode == I_TYPE);
  assign MemtoReg = (Opcode == LW);
  assign RegWrite = (Opcode == R_TYPE || Opcode == LW || Opcode == I_TYPE);
  assign MemRead  = (Opcode == LW);
  assign MemWrite = (Opcode == SW);
  assign ALUOp[0] = (Opcode == BR || Opcode == I_TYPE);
  assign ALUOp[1] = (Opcode == R_TYPE || Opcode == I_TYPE);
  assign Branch   = (Opcode == BR);

  // LoadSize e LoadSigned
  assign LoadSize = (Opcode == LW) ? (
                      (funct3 == 3'b000 || funct3 == 3'b100) ? 2'b10 : // LB / LBU
                      (funct3 == 3'b001 || funct3 == 3'b101) ? 2'b01 : // LH / LHU
                      (funct3 == 3'b010) ? 2'b00 :                     // LW
                      2'bxx
                    ) : 2'bxx;

  assign LoadSigned = (Opcode == LW) ? (
                        (funct3 == 3'b100 || funct3 == 3'b101) ? 1'b0 : 1'b1
                      ) : 1'bx;

endmodule
