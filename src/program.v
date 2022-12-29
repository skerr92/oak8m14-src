// SPDX-FileCopyrightText: © 2022 Seth Kerr <hello@oakdev.tech>
// Based on SPELL by Uri Shaked <https://github.com/wokwi/verispell>
// SPDX-License-Identifier: MIT

`default_nettype none

module program_ev(
    input wire [3:0] opcode, // 4 bit op code
    input [5:0] pc, // 6 bit wide program counter (64 bit)
    input [3:0] sp, // stack pointer
    input [7:0] top, // top of stack
    input [7:0] btop, // below top of stack
    input [7:0] pmem_in, // program memory in
    output reg [5:0] pc_plus, // program counter + 1
    output reg [3:0] sp_min, // stack pointer - 1
    output reg [7:0] sp_w_cnt, // number of times written to the stack
    output reg [7:0] new_top, // new top of stack
    output reg [7:0] new_btop, // new below top of stack
    output reg pmem_we, // program memory write enable
    output reg pmem_d_type, // type of memory write
    output reg [7:0] pmem_out, // program memory write data
    output reg [5:0] pmem_w_addr, // program memory write address
    output reg sleep, // sleep execution
    output reg stop // stop execution
);

always @(*) begin
    pc_plus = pc+8'h1;
    sp_min = sp;
    sp_w_cnt = 2'h0;
    new_top = 8'dx;
    new_btop = 8'dx;
    pmem_we = 1'b0;
    pmem_d_type = 1'b0;
    pmem_w_addr = 8'dx;
    pmem_out = 8'dx;
    sleep = 0;
    stop = 0;

    case (opcode)
    4'h0: begin // add src2 to src1 and store at top of stack
        new_top = btop + top;
        sp_w_cnt = 1;
        sp_min = sp - 1;
    end
    4'h1: begin // subtract src2 from src1 and store at top of stack
        new_top = btop - top;
        sp_w_cnt = 1;
        sp_min = sp - 1;
    end
    4'h2: begin // AND src2 and src1 and store at top of stack
        new_top = btop & top;
        sp_w_cnt = 1;
        sp_min = sp - 1;
    end
    4'h3: begin // OR src2 with src1 and store at top of stack
        new_top = btop | top;
        sp_w_cnt = 1;
        sp_min = sp - 1;
    end
    4'h4: begin // XOR src2 with src1 and store at top of stack
        new_top = btop ^ top;
        sp_w_cnt = 1;
        sp_min = sp - 1;
    end
    4'h5: begin // not src1 register value
        new_top = !top;
        sp_min = sp - 1;
    end
    4'h6: begin // move program counter to stack pointer
        pc_plus = top;
        sp_min = sp - 1;
    end
    4'h7: begin // write to program memory
        pmem_we = 1'b1;
        pmem_d_type = 1'b0;
        pmem_w_addr = top;
        pmem_out = btop;
        sp_min = sp - 2;
    end
    4'h8: begin // read from program memory
        new_top = pmem_in;
        sp_w_cnt = 1;
    end
    4'h9: begin // move src2 to src1
        new_top = btop;
        sp_w_cnt = 1;
    end
    4'hA: begin 
        new_top = top;
        sp_w_cnt = 1;
        sp_min = sp + 1;
    end
    4'hB: begin 
        new_top = btop;
        new_btop = top;
        sp_w_cnt = 2;
    end
    4'hC: begin 
        sp_min = sp - 2;
    end
    4'hE: sleep = 1'b1; // sleep
    4'hF: stop = 1'b1;
    default: begin
        new_top = opcode;
        sp_w_cnt = 1;
        sp_min = sp + 1;
      end
    endcase
end

endmodule