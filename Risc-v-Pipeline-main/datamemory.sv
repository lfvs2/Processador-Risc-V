module datamemory #(
    parameter DM_ADDRESS = 9,
    parameter DATA_W = 32
) (
    input logic clk,
    input logic MemRead,
    input logic MemWrite,
    input logic [DM_ADDRESS - 1:0] a,
    input logic [DATA_W - 1:0] wd,

    // Novos sinais
    input logic [1:0] LoadSize,    // 00: word, 01: half, 10: byte
    input logic       LoadSigned,  // 1: signed, 0: unsigned

    input logic [2:0] Funct3,      // ainda usado para escrita
    output logic [DATA_W - 1:0] rd
);

  logic [31:0] raddress;
  logic [31:0] waddress;
  logic [31:0] Datain;
  logic [31:0] Dataout;
  logic [ 3:0] Wr;

  // Registradores internos para captura sincronizada
  logic [1:0] load_size_reg;
  logic load_signed_reg;
  logic mem_read_reg;

  always_comb begin
    raddress = {{22{1'b0}}, a};
    waddress = {{22{1'b0}}, {a[8:2], 2'b00}};
    Datain = wd;
    Wr = 4'b0000;
    
    if (MemWrite) begin
      case (Funct3)
        3'b000: Wr = 4'b0001; // SB
        3'b001: Wr = 4'b0011; // SH
        3'b010: Wr = 4'b1111; // SW
        default: Wr = 4'b0000;
      endcase
    end
  end

  // Memória
  Memoria32Data mem32 (
      .raddress(raddress),
      .waddress(waddress),
      .Clk(~clk),
      .Datain(Datain),
      .Dataout(Dataout),
      .Wr(Wr)
  );

  // Registrando sinais para uso após latência
  always_ff @(posedge clk) begin
    load_size_reg   <= LoadSize;
    load_signed_reg <= LoadSigned;
    mem_read_reg    <= MemRead;
  end

  // Saída com máscara e extensão
  always_ff @(posedge clk) begin
    if (mem_read_reg) begin
      case (load_size_reg)
        2'b10: rd <= load_signed_reg ? {{24{Dataout[7]}}, Dataout[7:0]}  // LB
                                     : {24'b0, Dataout[7:0]};            // LBU
        2'b01: rd <= load_signed_reg ? {{16{Dataout[15]}}, Dataout[15:0]} // LH
                                     : {16'b0, Dataout[15:0]};            // LHU
        2'b00: rd <= Dataout;                                            // LW
        default: rd <= Dataout;
      endcase
    end
  end

endmodule
