`timescale 1ns / 1ps

module ALUController (
    //Inputs
    input logic [1:0] ALUOp,  // 2-bit opcode field from the Controller--00: LW/SW/AUIPC; 01:Branch; 10: Rtype/Itype; 11:JAL/LUI
    input logic [6:0] Funct7,  // bits 25 to 31 of the instruction
    input logic [2:0] Funct3,  // bits 12 to 14 of the instruction

    //Output
    output logic [3:0] Operation  // operation selection for ALU
);
    //Mudei pra facilitar e deixar mais bonitinho
  always_comb begin
        Operation = 4'b0000;

        casez ({ALUOp, Funct7, Funct3})
            // ADD e ADDI Funct3 = 000, Funct7 = 0000000 para ADD R-type; Funct7 é ? para ADDI
            // (ALUOp=10, Funct3=000)
            {2'b10, 7'b0000000, 3'b000}: Operation = 4'b0010; // ADD (R-type)
            {2'b11, 7'b???????, 3'b000}: Operation = 4'b0010; // ADDI (I-type, Funct7 não importa)

            // SUB (Funct3 = 000, Funct7 = 0100000)
            {2'b10, 7'b0100000, 3'b000}: Operation = 4'b0001; // SUB (R-type)

            // AND
            {2'b10, 7'b0000000, 3'b111}: Operation = 4'b0000; // AND (R-type)

            // OR 
            {2'b10, 7'b0000000, 3'b110}: Operation = 4'b0111; // OR (R-type)

            // XOR
            {2'b10, 7'b0000000, 3'b100}: Operation = 4'b0110; // XOR (R-type)

            // SLT / SLTI (Funct3 = 010, Funct7 = 0000000 para SLT R-type; Funct7 é '?' para SLTI)
            {2'b10, 7'b0000000, 3'b010}: Operation = 4'b1000; // SLT (R-type)
            {2'b11, 7'b???????, 3'b010}: Operation = 4'b1000; // SLTI (I-type)

            // SLLI 
            {2'b11, 7'b???????, 3'b001}: Operation = 4'b0011; // SLLI (I-type)

            // SRL / SRLI (Funct3 = 101, Funct7 = 0000000)
            {2'b11, 7'b0000000, 3'b101}: Operation = 4'b0100; // SRLI

            // SRA / SRAI (Funct3 = 101, Funct7 = 0100000)
            {2'b11, 7'b0100000, 3'b101}: Operation = 4'b0101; // SRA / SRAI

            // LW / SW / AUIPC (ALUOp == 2'b00)
            {2'b00, 7'b???????, 3'b???}: Operation = 4'b0010; // ADD

            // Branch (ALUOp == 2'b01)
            {2'b01, 7'b???????, 3'b???}: Operation = 4'b1001; // BEQ (comparação)
            
            // JAL / LUI (ALUOp == 2'b11)
            {2'b11, 7'b???????, 3'b???}: Operation = 4'b0010; // ADD (para JAL e outras operações que possam usar a ALU)

            default: Operation = 4'b0000; // Operaçao de AND, ou outra operação "segura"
        endcase
    end
endmodule
