`timescale 1ns / 1ps

import Pipe_Buf_Reg_PKG::*;

module Datapath #(
    parameter PC_W = 9,  // Program Counter
    parameter INS_W = 32,  // Instruction Width
    parameter RF_ADDRESS = 5,  // Register File Address
    parameter DATA_W = 32,  // Data WriteData
    parameter DM_ADDRESS = 9,  // Data Memory Address
    parameter ALU_CC_W = 4,  // ALU Control Code Width
    parameter HALT = 7'b1111111
) (
    input  logic                 clk,
    reset,
    RegWrite,
    MemtoReg,  // Register file writing enable   // Memory or ALU MUX
    ALUSrc,
    MemWrite,  // Register file or Immediate MUX // Memroy Writing Enable
    MemRead,  // Memroy Reading Enable
    Branch,  // Branch Enable
    input  logic [          1:0] ALUOp,
    input  logic [ALU_CC_W -1:0] ALU_CC,         // ALU Control Code ( input of the ALU )

    input logic HaltSignal,

    output logic HaltedState,

    output logic [          6:0] opcode,
    output logic [          6:0] Funct7,
    output logic [          2:0] Funct3,
    output logic [          1:0] ALUOp_Current,
    output logic [   DATA_W-1:0] WB_Data,        //Result After the last MUX

    // Para depuração no tesbench:
    output logic [4:0] reg_num,  //número do registrador que foi escrito
    output logic [DATA_W-1:0] reg_data,  //valor que foi escrito no registrador
    output logic reg_write_sig,  //sinal de escrita no registrador

    output logic wr,  // write enable
    output logic reade,  // read enable
    output logic [DM_ADDRESS-1:0] addr,  // address
    output logic [DATA_W-1:0] wr_data,  // write data
    output logic [DATA_W-1:0] rd_data  // read data
);

  logic [PC_W-1:0] PC, PCPlus4, Next_PC;
  logic [INS_W-1:0] Instr;
  logic [DATA_W-1:0] Reg1, Reg2;
  logic [DATA_W-1:0] ReadData;
  logic [DATA_W-1:0] SrcB, ALUResult;
  logic [DATA_W-1:0] ExtImm, BrImm, Old_PC_Four, BrPC;
  logic [DATA_W-1:0] WrmuxSrc;
  logic PcSel;  // mux select / flush signal
  logic [1:0] FAmuxSel;
  logic [1:0] FBmuxSel;
  logic [DATA_W-1:0] FAmux_Result;
  logic [DATA_W-1:0] FBmux_Result;
  logic Reg_Stall;  //1: PC fetch same, Register not update

  if_id_reg A;
  id_ex_reg B;
  ex_mem_reg C;
  mem_wb_reg D;

  logic halted;
  assign HaltedState = halted;

  always @(posedge clk) begin
    if (reset) begin
        PC <= 0;
    end
    else if (!halted) begin
        PC <= Next_PC;
    end
  end

  always @(posedge clk) begin 
    if (reset || PcSel) begin 
        A <= '0;
    end 
    else if (!halted && !Reg_Stall)begin 
        A.Curr_Pc <= PC;
        A.Curr_Instr <= Instr;
    end

  end

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      halted <= 1'b0;  // Reset assíncrono sai do estado HALT
      // Reset de outros registradores...
      PC <= 0;
      A <= '{default:0};
      B <= '{default:0};
      C <= '{default:0};
      D <= '{default:0};
    end else if (HaltSignal) begin
      halted <= 1'b1;  // Entra em estado HALT
    end
  end

  // next PC
  adder #(9) pcadd (
      PC,
      9'b100,
      PCPlus4
  );
  mux2 #(9) pcmux (
      PCPlus4,
      BrPC[PC_W-1:0],
      PcSel,
      Next_PC
  );
  flopr #(9) pcreg (
      clk,
      reset,
      Next_PC,
      Reg_Stall,
      PC
  );
  instructionmemory instr_mem (
      clk,
      PC,
      Instr
  );

  // IF_ID_Reg A;
  always @(posedge clk) begin
    if ((reset) || (PcSel) || (opcode == HALT))   // initialization or flush
        begin
      A.Curr_Pc <= 0;
      A.Curr_Instr <= 0;
    end

    else if (opcode == HALT)begin 
      A.Curr_Pc <= PC;
      A.Curr_Instr <= 32'h00000013;
    end

    else if (!Reg_Stall && !halted)    // stall
    begin
        A.Curr_Pc <= PC;
        A.Curr_Instr <= Instr;
    end
  end

  //--// The Hazard Detection Unit
  HazardDetection detect (
      A.Curr_Instr[19:15],
      A.Curr_Instr[24:20],
      B.rd,
      B.MemRead,
      Reg_Stall
  );

  // //Register File
  assign opcode = A.Curr_Instr[6:0];

  RegFile rf (
      clk,
      reset,
      D.RegWrite,
      D.rd,
      A.Curr_Instr[19:15],
      A.Curr_Instr[24:20],
      WrmuxSrc,
      Reg1,
      Reg2
  );

  assign reg_num = D.rd;
  assign reg_data = WrmuxSrc;
  assign reg_write_sig = D.RegWrite;

  // //sign extend
  imm_Gen Ext_Imm (
      A.Curr_Instr,
      ExtImm
  );

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      // Reset completo (assíncrono)
      B.ALUSrc <= 0;
      B.MemtoReg <= 0;
      B.RegWrite <= 0;
      B.MemRead <= 0;
      B.MemWrite <= 0;
      B.ALUOp <= 0;
      B.Branch <= 0;
      B.Curr_Pc <= 0;
      B.RD_One <= 0;
      B.RD_Two <= 0;
      B.RS_One <= 0;
      B.RS_Two <= 0;
      B.rd <= 0;
      B.ImmG <= 0;
      B.func3 <= 0;
      B.func7 <= 0;
      B.Curr_Instr <= 0;
    end
    else begin
        if (Reg_Stall || PcSel) begin
          // Stall por hazard ou branch taken - injeta NOP
          B.ALUSrc <= 0;
          B.MemtoReg <= 0;
          B.RegWrite <= 0;
          B.MemRead <= 0;
          B.MemWrite <= 0;
          B.ALUOp <= 0;
          B.Branch <= 0;
          // Mantém os valores dos registradores para evitar corrupção
          B.Curr_Pc <= B.Curr_Pc;
          B.RD_One <= B.RD_One;
          B.RD_Two <= B.RD_Two;
          B.RS_One <= B.RS_One;
          B.RS_Two <= B.RS_Two;
          B.rd <= B.rd;
          B.ImmG <= B.ImmG;
          B.func3 <= B.func3;
          B.func7 <= B.func7;
          B.Curr_Instr <= 32'h00000013; // NOP explícito
        end
        else if (halted) begin
          // Congelamento durante HALT - mantém tudo inalterado
          B.ALUSrc <= B.ALUSrc;
          B.MemtoReg <= B.MemtoReg;
          B.RegWrite <= B.RegWrite;
          B.MemRead <= B.MemRead;
          B.MemWrite <= B.MemWrite;
          B.ALUOp <= B.ALUOp;
          B.Branch <= B.Branch;
          B.Curr_Pc <= B.Curr_Pc;
          B.RD_One <= B.RD_One;
          B.RD_Two <= B.RD_Two;
          B.RS_One <= B.RS_One;
          B.RS_Two <= B.RS_Two;
          B.rd <= B.rd;
          B.ImmG <= B.ImmG;
          B.func3 <= B.func3;
          B.func7 <= B.func7;
          B.Curr_Instr <= B.Curr_Instr;
        end
        else begin
          // Operação normal
          B.ALUSrc <= ALUSrc;
          B.MemtoReg <= MemtoReg;
          B.RegWrite <= RegWrite;
          B.MemRead <= MemRead;
          B.MemWrite <= MemWrite;
          B.ALUOp <= ALUOp;
          B.Branch <= Branch;
          B.Curr_Pc <= A.Curr_Pc;
          B.RD_One <= Reg1;
          B.RD_Two <= Reg2;
          B.RS_One <= A.Curr_Instr[19:15];
          B.RS_Two <= A.Curr_Instr[24:20];
          B.rd <= A.Curr_Instr[11:7];
          B.ImmG <= ExtImm;
          B.func3 <= A.Curr_Instr[14:12];
          B.func7 <= A.Curr_Instr[31:25];
          B.Curr_Instr <= A.Curr_Instr;
        end
    end
  end

  //--// The Forwarding Unit
  ForwardingUnit forunit (
      B.RS_One,
      B.RS_Two,
      C.rd,
      D.rd,
      C.RegWrite,
      D.RegWrite,
      FAmuxSel,
      FBmuxSel
  );

  // // //ALU
  assign Funct7 = B.func7;
  assign Funct3 = B.func3;
  assign ALUOp_Current = B.ALUOp;

  mux4 #(32) FAmux (
      B.RD_One,
      WrmuxSrc,
      C.Alu_Result,
      B.RD_One,
      FAmuxSel,
      FAmux_Result
  );
  mux4 #(32) FBmux (
      B.RD_Two,
      WrmuxSrc,
      C.Alu_Result,
      B.RD_Two,
      FBmuxSel,
      FBmux_Result
  );
  mux2 #(32) srcbmux (
      FBmux_Result,
      B.ImmG,
      B.ALUSrc,
      SrcB
  );
  alu alu_module (
      FAmux_Result,
      SrcB,
      ALU_CC,
      ALUResult
  );
  BranchUnit #(9) brunit (
      B.Curr_Pc,
      B.ImmG,
      B.Branch,
      ALUResult,
      BrImm,
      Old_PC_Four,
      BrPC,
      PcSel
  );

  // EX_MEM_Reg C;
  always @(posedge clk) begin
    if (reset || (opcode == HALT))   // initialization
        begin
      C.RegWrite <= 0;
      C.MemtoReg <= 0;
      C.MemRead <= 0;
      C.MemWrite <= 0;
      C.Pc_Imm <= 0;
      C.Pc_Four <= 0;
      C.Imm_Out <= 0;
      C.Alu_Result <= 0;
      C.RD_Two <= 0;
      C.rd <= 0;
      C.func3 <= 0;
      C.func7 <= 0;
    end else begin
      C.RegWrite <= B.RegWrite && !HaltSignal;
      C.MemtoReg <= B.MemtoReg;
      C.MemRead <= B.MemRead && !HaltSignal;
      C.MemWrite <= B.MemWrite && !HaltSignal;
      C.Pc_Imm <= BrImm;
      C.Pc_Four <= Old_PC_Four;
      C.Imm_Out <= B.ImmG;
      C.Alu_Result <= ALUResult;
      C.RD_Two <= FBmux_Result;
      C.rd <= B.rd;
      C.func3 <= B.func3;
      C.func7 <= B.func7;
      C.Curr_Instr <= B.Curr_Instr;  // debug tmp
    end
  end

  // // // // Data memory 
  datamemory data_mem (
      clk,
      C.MemRead && !halted,
      C.MemWrite && !halted,
      C.Alu_Result[8:0],
      C.RD_Two,
      C.func3,
      ReadData
  );

  assign wr = C.MemWrite && !halted;
  assign reade = C.MemRead && !halted;
  assign addr = C.Alu_Result[8:0];
  assign wr_data = C.RD_Two;
  assign rd_data = ReadData;

  // MEM_WB_Reg D;
  always @(posedge clk) begin
    if (reset || (opcode == HALT))   // initialization
        begin
      D.RegWrite <= 0;
      D.MemtoReg <= 0;
      D.Pc_Imm <= 0;
      D.Pc_Four <= 0;
      D.Imm_Out <= 0;
      D.Alu_Result <= 0;
      D.MemReadData <= 0;
      D.rd <= 0;
    end else begin
      D.RegWrite <= C.RegWrite;
      D.MemtoReg <= C.MemtoReg;
      D.Pc_Imm <= C.Pc_Imm;
      D.Pc_Four <= C.Pc_Four;
      D.Imm_Out <= C.Imm_Out;
      D.Alu_Result <= C.Alu_Result;
      D.MemReadData <= ReadData;
      D.rd <= C.rd;
      D.Curr_Instr <= C.Curr_Instr;  //Debug Tmp
    end
  end

  //--// The LAST Block
  mux2 #(32) resmux (
      D.Alu_Result,
      D.MemReadData,
      D.MemtoReg,
      WrmuxSrc
  );

  assign WB_Data = WrmuxSrc;
  assign reg_num = D.rd;
  assign reg_data = WrmuxSrc;
  assign reg_write_sig = D.RegWrite && !halted;

endmodule