`timescale 1ns / 1ps

module riscv #(
    parameter DATA_W = 32
) (
    input logic clk,
    reset,  // clock and reset signals
    output logic [31:0] WB_Data,  // The ALU_Result

    output logic [4:0] reg_num,
    output logic [31:0] reg_data,
    output logic reg_write_sig,

    output logic wr,
    output logic rd,
    output logic [8:0] addr,
    output logic [DATA_W-1:0] wr_data,
    output logic [DATA_W-1:0] rd_data
);

  logic [6:0] opcode;
  logic ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch;
  logic [1:0] ALUop;
  logic [1:0] ALUop_Reg;
  logic [6:0] Funct7;
  logic [2:0] Funct3;
  logic [3:0] Operation;

  // Novos sinais
  logic [1:0] LoadSize;
  logic       LoadSigned;

  Controller c (
      .Opcode(opcode),
      .funct3(Funct3),        // <-- novo
      .ALUSrc(ALUSrc),
      .MemtoReg(MemtoReg),
      .RegWrite(RegWrite),
      .MemRead(MemRead),
      .MemWrite(MemWrite),
      .ALUOp(ALUop),
      .Branch(Branch),
      .LoadSize(LoadSize),     // <-- novo
      .LoadSigned(LoadSigned)  // <-- novo
  );

  ALUController ac (
      .ALUOp(ALUop_Reg),
      .Funct7(Funct7),
      .Funct3(Funct3),
      .Operation(Operation)
  );

  Datapath dp (
      .clk(clk),
      .reset(reset),
      .RegWrite(RegWrite),
      .MemtoReg(MemtoReg),
      .ALUsrc(ALUSrc),
      .MemWrite(MemWrite),
      .MemRead(MemRead),
      .Branch(Branch),
      .ALUOp(ALUop),
      .ALU_CC(Operation),
      .opcode(opcode),
      .Funct7(Funct7),
      .Funct3(Funct3),
      .ALUOp_Current(ALUop_Reg),
      .WB_Data(WB_Data),
      .reg_num(reg_num),
      .reg_data(reg_data),
      .reg_write_sig(reg_write_sig),
      .wr(wr),
      .addr(addr),
      .wr_data(wr_data),
      .rd_data(rd_data),

      // Novos sinais conectados ao datapath
      .LoadSize(LoadSize),
      .LoadSigned(LoadSigned)
  );

endmodule
