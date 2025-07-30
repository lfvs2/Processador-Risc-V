`timescale 1ns / 1ps

module ALUController (
    input logic [1:0] ALUOp,     // 00: LW/SW/AUIPC, 01: Branches, 10: R-type/I-type
    input logic [6:0] Funct7,    // bits 25-31
    input logic [2:0] Funct3,    // bits 12-14
    output logic [3:0] Operation // ALU operation code
);

// this organization clearify more the code, splitting by ALUOp
  always_comb begin
    case (ALUOp)
      2'b00: begin 
        Operation = 4'b0010; // LW, SW, AUIPC — uses ADD
      end
      
      2'b01: begin // Branches
        case (Funct3)
          3'b000: Operation = 4'b1000; // BEQ
          3'b001: Operation = 4'b1001; // BNE
          3'b100: Operation = 4'b1100; // BLT
          3'b101: Operation = 4'b1101; // BGE
          default: Operation = 4'b0000;
        endcase
      end
      
      2'b10: begin // R-type and I-type (como ADD, SUB, ADDI, SLTI, etc.)
        case (Funct3)
          3'b000: begin
            if (Funct7 == 7'b0100000)
              Operation = 4'b0001; // SUB
            else
              Operation = 4'b0010; // ADD / ADDI
          end
		
          3'b001: Operation = 4'b0011; // SLL / SLLI
          3'b010: Operation = 4'b1000; // SLT / SLTI
          3'b100: Operation = 4'b0110; // XOR / XORI
          3'b101: begin
            if (Funct7 == 7'b0100000)
              Operation = 4'b0101; // SRA / SRAI
            else
              Operation = 4'b0100; // SRL / SRLI
          end
          3'b110: Operation = 4'b0111; // OR / ORI
          3'b111: Operation = 4'b0000; // AND / ANDI
          default: Operation = 4'b0000;
        endcase
      end

      default: begin // default for secure the process, in case of don't recognizes any other ALUOp.
        Operation = 4'b0000;
      end
    endcase
  end

endmodule
