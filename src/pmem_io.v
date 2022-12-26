// SPDX-FileCopyrightText: © 2022 Seth Kerr <hello@oakdev.tech>
// Based on SPELL by Uri Shaked <https://github.com/wokwi/verispell>
// SPDX-License-Identifier: MIT

`default_nettype none
//
`timescale 1ns / 1ps

module pmem_io (
    input wire reset,
    input wire clock,
    input wire select,
    input wire [7:0] addr,
    input wire [7:0] data_in,
    input wire write,
    output reg [7:0] data_out,
    output reg data_ready,

    /* IO */
    input  wire [7:0] io_in,
    output reg  [7:0] io_out,
    output reg  [7:0] io_oeb   // out enable bar (low active)
);

  localparam REG_PIN = 8'h36;
  localparam REG_DDR = 8'h37;
  localparam REG_PORT = 8'h38;

  reg past_write;

  always @(posedge clock) begin
    if (reset) begin
      io_out <= 8'b00000000;
      io_oeb <= 8'b11111111;
      data_out <= 8'b0;
      data_ready <= 1'b0;
      past_write <= 1'b0;
    end else begin
      past_write <= select & write;
      if (select) begin
        data_out   <= 8'b0;
        data_ready <= 1'b1;

        case (addr)
          REG_PIN: begin
            if (write) begin
              if (!past_write) io_out <= io_out ^ data_in;
            end else begin
              data_out <= io_in;
            end
          end
          REG_DDR: begin
            if (write) begin
              io_oeb <= ~data_in;
            end else begin
              data_out <= ~io_oeb;
            end
          end
          REG_PORT: begin
            if (write) begin
              io_out <= data_in;
            end else begin
              data_out <= io_out;
            end
          end
        endcase
      end else data_ready <= 1'b0;
    end
  end
endmodule