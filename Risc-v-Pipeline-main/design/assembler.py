"""
Author: Nathalia Barbosa (@nathaliafab)
Date: 2023-06-16

This script translates instructions from a given file into a format readable by the instruction memory.
(It basically functions as an assembler.)

> The instructions to be translated are stored in a file named "instructions.txt"
> The translated instructions will be written to a file named "instruction.mif"

The instructions MUST follow the following formats, with one instruction per line:

<instruction> <register>,<register>,<register>
<instruction> <register>,<register>,<immediate>
<instruction> <register>,<offset>(<register>)
<instruction> <register>,<immediate>

~ Otherwise, it won't work. ;)

Example of a valid instruction file (content delimited by ```):
```
sub x6,x6,x1
addi x1,x0,8
lw x9,0(x0)
auipc x6,3
```
"""

import os

# bits 0-6
opcode = {
    "lui": "0110111",
    "auipc": "0010111",
    "jal": "1101111",
    "jalr": "1100111",
    "beq": "1100011",
    "bne": "1100011",
    "blt": "1100011",
    "bge": "1100011",
    "bltu": "1100011",
    "bgeu": "1100011",
    "lb": "0000011",
    "lh": "0000011",
    "lw": "0000011",
    "lbu": "0000011",
    "lhu": "0000011",
    "sb": "0100011",
    "sh": "0100011",
    "sw": "0100011",
    "addi": "0010011",
    "slti": "0010011",
    "sltiu": "0010011",
    "xori": "0010011",
    "ori": "0010011",
    "andi": "0010011",
    "slli": "0010011",
    "srli": "0010011",
    "srai": "0010011",
    "add": "0110011",
    "sub": "0110011",
    "sll": "0110011",
    "slt": "0110011",
    "sltu": "0110011",
    "xor": "0110011",
    "srl": "0110011",
    "sra": "0110011",
    "or": "0110011",
    "and": "0110011",
}

# bits 12-14
funct3 = {
    "jalr": "000",
    "beq": "000",
    "bne": "001",
    "blt": "100",
    "bge": "101",
    "bltu": "110",
    "bgeu": "111",
    "lb": "000",
    "lh": "001",
    "lw": "010",
    "lbu": "100",
    "lhu": "101",
    "sb": "000",
    "sh": "001",
    "sw": "010",
    "addi": "000",
    "slti": "010",
    "sltiu": "011",
    "xori": "100",
    "ori": "110",
    "andi": "111",
    "slli": "001",
    "srli": "101",
    "srai": "101",
    "add": "000",
    "sub": "000",
    "sll": "001",
    "slt": "010",
    "sltu": "011",
    "xor": "100",
    "srl": "101",
    "sra": "101",
    "or": "110",
    "and": "111",
}

# bits 25-31
funct7 = {
    "slli": "0000000",
    "srli": "0000000",
    "srai": "0100000",
    "add": "0000000",
    "sub": "0100000",
    "sll": "0000000",
    "slt": "0000000",
    "sltu": "0000000",
    "xor": "0000000",
    "srl": "0000000",
    "sra": "0100000",
    "or": "0000000",
    "and": "0000000",
}


def create_file(file_name):
    with open(file_name, "w") as file:
        file.write("DEPTH = 65536;\nWIDTH = 8;\nADDRESS_RADIX = DEC;\nDATA_RADIX = BIN;\nCONTENT\nBEGIN\n\n")


def read_file(file_name):
    try:
        with open(file_name, "r") as file:
            instructions = file.readlines()
        if not instructions:
            raise Exception
    except Exception:
        print(f"The file '{file_name}' could not be read. Make sure it exists and is not empty.\n")
        exit(1)
    return instructions


def write_instruction(file_name, index, chunk, instr):
    if (int(index) % 4 != 0):
        instr = ""
    else:
        instr = "		-- " + instr.rstrip("\n")
    with open(file_name, "a") as file:
        file.write(str(index) + ": " + chunk + ";" + instr + "\n")
        if (int(index) % 4 == 3):
            file.write("\n")


def end_file(file_name):
    with open(file_name, "a") as file:
        file.write("END;")


def sbin(value, bits=32):
    value = int(value)
    if value >= 0:
        return format(value, f'0{bits}b')
    else:
        return format((1 << bits) + value, f'0{bits}b')


def translate(instruction):
    try:
        binary = translate_instruction(instruction)
        success = True
    except Exception:
        binary = ""
        success = False
    return binary, success


def translate_instruction(instruction):
    instr = instruction.split(" ")[0]
    rd = instruction.split(" ")[1].split(",")[0]
    rd = bin(int(rd[1:]))[2:].zfill(5)

    if instr in ["lui", "auipc"]:
        imm = instruction.split(" ")[1].split(",")[1]
        imm = sbin(imm, 20)
        binary = imm + rd + opcode[instr]

    elif instr == "jal":
        imm = instruction.split(" ")[1].split(",")[1]
        imm = sbin(imm, 21)[::-1]
        bit20 = imm[20]
        bit10to1 = (imm[1:11])[::-1]
        bit11 = imm[11]
        bit19to12 = (imm[12:20])[::-1]
        imm = bit20 + bit19to12 + bit11 + bit10to1
        binary = imm + rd + opcode[instr]

    elif instr == "jalr":
        rs1 = instruction.split(" ")[1].split(",")[1]
        rs1 = bin(int(rs1[1:]))[2:].zfill(5)
        imm = instruction.split(" ")[1].split(",")[2]
        imm = sbin(imm, 12)
        binary = imm + rs1 + funct3[instr] + rd + opcode[instr]

    elif instr in ["beq", "bne", "blt", "bge", "bltu", "bgeu"]:
        rs1 = bin(int(instruction.split(" ")[1].split(",")[0][1:]))[2:].zfill(5)
        rs2 = bin(int(instruction.split(" ")[1].split(",")[1][1:]))[2:].zfill(5)
        imm = sbin(instruction.split(" ")[1].split(",")[2], 13)[::-1]
        bit12 = imm[12]
        bit10to5 = (imm[5:11])[::-1]
        bit4to1 = (imm[1:5])[::-1]
        bit11 = imm[11]
        binary = bit12 + bit10to5 + rs2 + rs1 + funct3[instr] + bit4to1 + bit11 + opcode[instr]

    elif instr in ["lb", "lh", "lw", "lbu", "lhu"]:
        rs1 = instruction.split(" ")[1].split(",")[1]
        rs1 = rs1.split("(")[1].split(")")[0]
        rs1 = bin(int(rs1[1:]))[2:].zfill(5)
        imm = instruction.split(" ")[1].split(",")[1].split("(")[0]
        imm = sbin(imm, 12)
        binary = imm + rs1 + funct3[instr] + rd + opcode[instr]

    elif instr in ["sb", "sh", "sw"]:
        rs2 = bin(int(instruction.split(" ")[1].split(",")[0][1:]))[2:].zfill(5)
        rs1 = instruction.split(" ")[1].split(",")[1]
        rs1 = rs1.split("(")[1].split(")")[0]
        rs1 = bin(int(rs1[1:]))[2:].zfill(5)
        imm = sbin(instruction.split(" ")[1].split(",")[1].split("(")[0], 12)[::-1]
        bit11to5 = (imm[5:12])[::-1]
        bit4to0 = (imm[0:5])[::-1]
        binary = bit11to5 + rs2 + rs1 + funct3[instr] + bit4to0 + opcode[instr]

    elif instr in ["addi", "slti", "sltiu", "xori", "ori", "andi"]:
        rs1 = bin(int(instruction.split(" ")[1].split(",")[1][1:]))[2:].zfill(5)
        imm = sbin(instruction.split(" ")[1].split(",")[2], 12)
        binary = imm + rs1 + funct3[instr] + rd + opcode[instr]

    elif instr in ["slli", "srli", "srai"]:
        rs1 = bin(int(instruction.split(" ")[1].split(",")[1][1:]))[2:].zfill(5)
        shamt = sbin(instruction.split(" ")[1].split(",")[2], 5)
        binary = funct7[instr] + shamt + rs1 + funct3[instr] + rd + opcode[instr]

    elif instr in ["add", "sub", "sll", "slt", "sltu", "xor", "srl", "sra", "or", "and"]:
        rs1 = bin(int(instruction.split(" ")[1].split(",")[1][1:]))[2:].zfill(5)
        rs2 = bin(int(instruction.split(" ")[1].split(",")[2][1:]))[2:].zfill(5)
        binary = funct7[instr] + rs2 + rs1 + funct3[instr] + rd + opcode[instr]

    return binary


def main():
    instructions = read_file("instructions.txt")
    create_file("instruction.mif")

    for i in range(len(instructions)):
        binary, success = translate(instructions[i])
        if success:
            chunks = [binary[j:j+8] for j in range(0, len(binary), 8)][::-1]
            for j in range(len(chunks)):
                index = "{:03d}".format(i * 4 + j)
                write_instruction("instruction.mif", index, chunks[j], instructions[i])
        else:
            instr = instructions[i].rstrip("\n")
            line = i + 1
            print(f"Translation failed for instruction: \"{instr}\" on line {line}.\n")
            os.remove("instruction.mif")
            exit(2)

    end_file("instruction.mif")
    print("Assembly to machine code translation complete.\n")


main()
