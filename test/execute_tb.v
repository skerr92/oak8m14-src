// SPDX-FileCopyrightText: © 2022 Seth Kerr <hello@oakdev.tech>
// Based on SPELL by Uri Shaked <https://github.com/wokwi/verispell>
// SPDX-License-Identifier: MIT

`timescale 1ns / 1ps
//
`default_nettype none

module test_program_execute ();
  reg [7:0] stack[31:0];
  reg [3:0] opcode;
  reg [5:0] pc;
  reg [3:0] sp;
  reg [7:0] memory_input;
  wire [5:0] next_pc;
  wire [3:0] next_sp;
  wire [7:0] stack_write_count;
  wire [7:0] set_stack_top;
  wire [7:0] set_stack_belowtop;
  wire memory_write_en;
  wire memory_write_type_data;
  wire [5:0] memory_write_addr;
  wire [7:0] memory_write_data;
  wire sleep;

  // for VCD dump:
  wire [7:0] stack0 = stack[0];
  wire [7:0] stack1 = stack[1];

  program_ev exec (
      .opcode(opcode),
      .pc(pc),
      .sp(sp),
      .top(stack[sp-1]),
      .btop(stack[sp-2]),
      .pmem_in(memory_input),
      .pc_plus(next_pc),
      .sp_min(next_sp),
      .sp_w_cnt(stack_write_count),
      .new_top(set_stack_top),
      .new_btop(set_stack_belowtop),
      .pmem_we(memory_write_en),
      .pmem_d_type(memory_write_type_data),
      .pmem_w_addr(memory_write_addr),
      .pmem_out(memory_write_data),
      .sleep(sleep)
  );

  initial begin
    opcode = 4'hD;
    pc = 0;
    sp = 2;
    memory_input = 8'h42;
    stack[0] = 15;
    stack[1] = 10;

    // Arithemetic
    #10 opcode = 4'h0;
    #10 opcode = 4'h1;
    #10 opcode = 4'h2;
    #10 opcode = 4'h3;
    #10 opcode = 4'h4;
    // Stack
    #10 opcode = 4'hB;
    #10 opcode = 4'hA;
    // Flow control
    #10 opcode = 4'h6;
    // I/O
    #10 opcode = 4'h8;
    #10 opcode = 4'h5;
    #10 opcode = 4'h7;
    // Misc
    #10 opcode = 4'hE;
    #10;
  end

  initial begin
    $dumpfile("program_tb.vcd");
    $dumpvars(0, test_program_execute);
  end
endmodule