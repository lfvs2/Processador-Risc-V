`timescale 1ns / 1ps

module alu#(
        parameter DATA_WIDTH = 32,
        parameter OPCODE_LENGTH = 4
        )
        (
        input logic [DATA_WIDTH-1:0]    SrcA,
        input logic [DATA_WIDTH-1:0]    SrcB,

        input logic [OPCODE_LENGTH-1:0]    Operation,
        output logic[DATA_WIDTH-1:0] ALUResult
        );
    
        always_comb begin
        case (Operation)
                4'b0000: ALUResult = SrcA & SrcB;                           // AND
                4'b0001: ALUResult = SrcA - SrcB;                           // SUB
                4'b0010: ALUResult = SrcA + SrcB;                           // ADD, ADDI, LW, SW
                4'b0011: ALUResult = SrcA << SrcB[4:0];                     // SLL, SLLI
                4'b0100: ALUResult = SrcA >> SrcB[4:0];                     // SRL, SRLI
                4'b0101: ALUResult = $signed(SrcA) >>> SrcB[4:0];           // SRA, SRAI
                4'b0110: ALUResult = SrcA ^ SrcB;                           // XOR
                4'b0111: ALUResult = SrcA | SrcB;                           // OR
                4'b1000: ALUResult = ($signed(SrcA) < $signed(SrcB)) ? 1 : 0; // SLT, SLTI
                4'b1001: ALUResult = (SrcA == SrcB) ? 1 : 0;                // BEQ (comparação)
		4'b1010: ALUResult = (SrcA == SrcB) ? 0 : 1;                // BNE (comparação)
		4'b1011: ALUResult = (SrcA < SrcB) ? 1 : 0;                // BLT (comparação)
		4'b1100: ALUResult = (SrcA >= SrcB) ? 1 : 0;                // BGE (comparação)
                default: ALUResult = 32'd0;                                 // valor padrão
        endcase
        end
endmodule

