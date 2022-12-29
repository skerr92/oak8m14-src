// SPDX-FileCopyrightText: © 2022 Seth Kerr <hello@oakdev.tech>
// Based on SPELL by Uri Shaked <https://github.com/wokwi/verispell>
// SPDX-License-Identifier: MIT

`timescale 1ns / 1ps
//
`default_nettype none

module test_pmem_dff ();
  reg reset;
  reg clock;
  reg select;
  reg [7:0] addr;
  reg [7:0] data_in;
  reg memory_type_data;
  reg write;
  wire [7:0] data_out;
  wire data_ready;

  pmem_dff mem (
      .reset(reset),
      .clock(clock),
      .select(select),
      .addr(addr),
      .data_in(data_in),
      .memory_type_data(memory_type_data),
      .write(write),
      .data_out(data_out),
      .data_ready(data_ready)
  );

  initial begin
    clock = 0;
    forever begin
      #5 clock = ~clock;
    end
  end

  initial begin
    reset  = 1;
    select = 0;
    #20 reset = 0;
    #10 reset = 0;

    // Write a byte to data location 5
    addr = 5;
    data_in = 42;
    write = 1;
    memory_type_data = 1;
    select = 1;
    #90 select = 0;
    data_in = 0;
    #10 reset = 0;

    // Read a byte from code location 5
    addr = 5;
    write = 0;
    memory_type_data = 0;
    select = 1;
    #80 `assert(data_ready, 1);
    `assert(data_out, 0);
    select = 0;
    #10 reset = 0;

    // Read a byte from data location 5
    addr = 5;
    write = 0;
    memory_type_data = 1;
    select = 1;
    #80 `assert(data_ready, 1);
    `assert(data_out, 42);
    select = 0;
    #10 reset = 0;

    // Read a byte from data location 6
    addr = 6;
    write = 0;
    memory_type_data = 1;
    select = 1;
    #80 `assert(data_ready, 1);
    `assert(data_out, 0);
    select = 0;
    #10 reset = 0;

    // Write a byte to code location 5
    addr = 5;
    data_in = 99;
    write = 1;
    memory_type_data = 0;
    select = 1;
    #90 select = 0;
    data_in = 0;
    #10 reset = 0;

    // Read a byte from code location 5
    addr = 5;
    write = 0;
    memory_type_data = 0;
    select = 1;
    #80 `assert(data_ready, 1);
    `assert(data_out, 99);
    select = 0;
    #10 reset = 0;

    // Read a byte from data location 5
    addr = 5;
    write = 0;
    memory_type_data = 1;
    select = 1;
    #80 `assert(data_ready, 1);
    `assert(data_out, 42);
    select = 0;
    #10 reset = 0;

    $finish();
  end

  initial begin
    $dumpfile("pmem_dff_tb.vcd");
    $dumpvars(0, test_pmem_dff);
  end
endmodule