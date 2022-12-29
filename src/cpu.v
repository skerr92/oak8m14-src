// SPDX-FileCopyrightText: © 2022 Seth Kerr <hello@oakdev.tech>
// Based on SPELL by Uri Shaked <https://github.com/wokwi/verispell>
// SPDX-License-Identifier: MIT

module oak8m(
  input wire reset,
  input wire clk,

  // Logic anaylzer
  input wire i_la_write,
  input wire [6:0] i_la_addr,
  input wire [7:0] i_la_data,
  input wire i_la_wb_disable,
  output wire [31:0] la_data_out,

  // wishbone interface
  input  wire        i_wb_cyc,   // wishbone transaction
  input  wire        i_wb_stb,   // strobe
  input  wire        i_wb_we,    // write enable
  input  wire [31:0] i_wb_addr,  // address
  input  wire [31:0] i_wb_data,  // incoming data
  output wire        o_wb_ack,   // request is completed 
  output reg  [31:0] o_wb_data,  // output data

  // GPIO
 input  wire [7:0] io_in,
  output wire [7:0] io_out,
  output wire [7:0] io_oeb,  // out enable bar (low active)

  // SRAM wishbone controller
  output wire        rambus_wb_clk_o,   // clock, must run at system clock
  output wire        rambus_wb_rst_o,   // reset
  output wire        rambus_wb_stb_o,   // write strobe
  output wire        rambus_wb_cyc_o,   // cycle
  output wire        rambus_wb_we_o,    // write enable
  output wire [ 3:0] rambus_wb_sel_o,   // write word select
  output wire [31:0] rambus_wb_dat_o,   // ram data out
  output wire [ 9:0] rambus_wb_addr_o,  // 8 bit address
  input  wire        rambus_wb_ack_i,   // ack
  input  wire [31:0] rambus_wb_dat_i,   // ram data in

  // interrupt
  output wire interrupt
);

localparam fetch = 3'b000;
localparam store = 3'b001;
localparam ex    = 3'b010;
localparam str   = 3'b011;
localparam data  = 3'b100;
localparam slp = 3'b101;

localparam REG_PC = 24'h000;
localparam REG_SP = 24'h004;
localparam REG_EXEC = 24'h008;
localparam REG_CTRL = 24'h00c;
localparam REG_CYCLES_PER_MS = 24'h010;
localparam REG_STACK_TOP = 24'h014;
localparam REG_STACK_PUSH = 24'h018;
localparam REG_INT_ENABLE = 24'h20;
localparam REG_INT = 24'h24;

localparam INTR_SLEEP = 0;
localparam INTR_STOP = 1;
localparam INTR_COUNT = 2;

reg [2:0] state;
reg [5:0] pc;
reg [3:0] sp;
reg [3:0] opcode;
reg [7:0] pmem_in;
reg [7:0] stack [15:0];

wire [5:0] pc_plus;
wire [3:0] sp_min;
wire [7:0] sp_w_cnt;
wire [7:0] new_top;
wire [7:0] new_btop;
wire pmem_we;
wire pmem_d_type;
wire [5:0] pmem_w_addr;
wire [7:0] pmem_out;
wire sleep;
wire stop;

 // Interrupts
reg [INTR_COUNT-1:0] intr;
reg [INTR_COUNT-1:0] intr_enable;
reg edge_interrupts;
wire level_interrupt = |(intr & intr_enable);
reg prev_level_interrupt;
assign interrupt = edge_interrupts ? (!prev_level_interrupt && level_interrupt) : level_interrupt;

reg single_step;

wire [3:0] top_index = sp - 1;
wire [7:0] top = stack[top_index]; // current stack

// Memory related registers
reg sram_enable;
reg mem_select;
reg mem_type_data;
reg [7:0] mem_addr;
reg [7:0] mem_write_value;
reg mem_write_en;
wire [7:0] mem_read_value;
wire mem_data_ready;

// Delay related registers
reg [23:0] cycles_per_ms;
//reg [23:0] delay_cycles;
//reg [7:0] delay_counter;

// Wishbone registers
reg wb_read_ack;
reg wb_write_ack;
assign o_wb_ack = wb_read_ack | wb_write_ack;
wire wb_read = i_wb_stb && i_wb_cyc && !i_wb_we;
wire wb_write = i_wb_stb && i_wb_cyc && i_wb_we && !i_la_wb_disable;
wire [23:0] wb_addr = i_wb_addr[23:0];
reg prev_reg_write;

// Combined logic analyzer + wishbone write interface
wire reg_write = wb_write | i_la_write;
wire [6:0] reg_write_addr = i_la_write ? i_la_addr : i_wb_addr[6:0];
wire [31:0] reg_write_data = i_la_write ? {24'b0, i_la_data} : i_wb_data;

// RAM bus clock and reset
assign rambus_wb_clk_o = clk;
assign rambus_wb_rst_o = reset;

// Logic Analyzer Connections:
// [       Stack Top        | State  |      SP   |  Opcode   |         PC            ]
// [ 31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00 ]

assign la_data_out = {top, state, sp, opcode, pc};

// Debug stuff
reg [63:0] state_name;

always @(*) begin
  case (state)
    fetch: state_name <= "Fetch";
    ex: state_name <= "Execute";
    str: state_name <= "Store";
    slp: state_name <= "Sleep";
    data: state_name <= "FetchData";
    default: state_name <= "Invalid";
  endcase
end

program_ev program( // program evaluation
  .opcode(opcode),
  .pc(pc),
  .sp(sp),
  .top(top),
  .btop(stack[sp-2]),
  .pmem_in(pmem_in),
  .pc_plus(pc_plus),
  .sp_min(sp_min),
  .sp_w_cnt(sp_w_cnt),
  .new_top(new_top),
  .new_btop(new_btop),
  .pmem_we(pmem_we),
  .pmem_d_type(pmem_d_type),
  .pmem_w_addr(pmem_w_addr),
  .pmem_out(pmem_out),
  .sleep(sleep),
  .stop(stop)
);

pmem mem (
      .reset(reset),
      .clock(clk),
      .sram_enable(sram_enable),
      .select(mem_select),
      .addr(mem_addr),
      .data_in(mem_write_value),
      .memory_type_data(mem_type_data),
      .write(mem_write_en),
      .data_out(mem_read_value),
      .data_ready(mem_data_ready),
      // IO
      .io_in(io_in),
      .io_out(io_out),
      .io_oeb(io_oeb),
      // OpenRAM
      .sram_stb_o(rambus_wb_stb_o),
      .sram_cyc_o(rambus_wb_cyc_o),
      .sram_we_o(rambus_wb_we_o),
      .sram_sel_o(rambus_wb_sel_o),
      .sram_dat_o(rambus_wb_dat_o),
      .sram_addr_o(rambus_wb_addr_o),
      .sram_ack_i(rambus_wb_ack_i),
      .sram_dat_i(rambus_wb_dat_i)
  );

  function is_data_opcode(input [3:0] opcode);
    is_data_opcode = (opcode == 4'h7 || opcode == 4'h8);
  endfunction

  // Wishbone reads
  always @(posedge clk) begin
    if (reset) begin
      o_wb_data   <= 0;
      wb_read_ack <= 0;
    end else if (wb_read) begin
      o_wb_data <= 0;
      case (wb_addr)
        REG_PC: o_wb_data <= {24'b0, pc};
        REG_SP: o_wb_data <= {27'b0, sp};
        REG_EXEC: o_wb_data <= {24'b0, opcode};
        REG_CTRL: begin
          o_wb_data <= {28'b0, edge_interrupts, sram_enable, single_step, state != slp};
        end
        REG_CYCLES_PER_MS: o_wb_data <= {8'b0, cycles_per_ms};
        REG_STACK_TOP: o_wb_data <= {24'b0, top};
        REG_INT_ENABLE: o_wb_data[INTR_COUNT-1:0] <= intr_enable;
        REG_INT: o_wb_data[INTR_COUNT-1:0] <= intr;
        default: begin
          o_wb_data <= 32'b0;
        end
      endcase
      wb_read_ack <= 1;
    end else begin
      wb_read_ack <= 0;
    end
  end

  integer j;

  // Main logic
  always @(posedge clk) begin
    if (reset) begin
      state <= slp;
      pc    <= 0;
      sp    <= 0;
      for (j = 0; j < 32; j++) stack[j] = 0;
      opcode <= 0;
      mem_select <= 0;
      mem_write_en <= 0;
      wb_write_ack <= 0;
      prev_reg_write <= 0;
      sram_enable <= 0;
      intr <= 0;
      intr_enable <= 0;
      edge_interrupts <= 0;
      prev_level_interrupt <= 0;
      cycles_per_ms <= 24'd10000;  /* we assume a 10MHz clock */
    end else begin
      prev_reg_write <= reg_write;
      prev_level_interrupt <= level_interrupt;
      if (reg_write) begin
        case (reg_write_addr)
          REG_PC: pc <= reg_write_data[7:0];
          REG_SP: sp <= reg_write_data[4:0];
          REG_EXEC: begin
            if (state == slp) begin
              opcode = reg_write_data[7:0];
              state <= is_data_opcode(opcode) ? data : ex;
              single_step <= 1;
            end
          end
          REG_CTRL: begin
            if (reg_write_data[0] && state == slp) begin
              state <= fetch;
            end
            single_step <= reg_write_data[1];
            sram_enable <= reg_write_data[2];
            edge_interrupts <= reg_write_data[3];
          end
          REG_CYCLES_PER_MS: cycles_per_ms <= reg_write_data[23:0];
          REG_STACK_TOP: stack[top_index] <= reg_write_data[7:0];
          REG_STACK_PUSH:
          if (!prev_reg_write) begin
            stack[sp] <= reg_write_data[7:0];
            sp <= sp + 1;
          end
          REG_INT_ENABLE: intr_enable <= reg_write_data[INTR_COUNT-1:0];
          REG_INT: intr <= intr & ~reg_write_data[INTR_COUNT-1:0];
        endcase
        if (wb_write) wb_write_ack <= 1;
      end else begin
        wb_write_ack <= 0;
        case (state)
          fetch: begin
            // Read next instruction from code memory
            mem_select <= 1;
            mem_type_data <= 0;
            mem_addr <= pc;
            mem_write_en <= 0;
            if (mem_select && mem_data_ready) begin
              mem_select <= 0;
              opcode = mem_read_value;
              state <= is_data_opcode(opcode) ? data : ex;
            end
          end
          data: begin
            // Read data for instruction from either code or data memory
            mem_select <= 1;
            mem_type_data <= (opcode == 4'h8) ? 1'b1 : 1'b0;
            mem_addr <= top;
            mem_write_en <= 0;
            if (mem_select && mem_data_ready) begin
              mem_select <= 0;
              pmem_in <= mem_read_value;
              state <= ex;
            end
          end
          ex: begin
            // Execute a single instruction
            pc <= pc_plus;
            sp <= sp_min;
            mem_type_data <= pmem_d_type;
            mem_addr <= pmem_w_addr;
            mem_write_value <= pmem_out;
            if (slp) intr[INTR_SLEEP] = 1'b1;
            if (stop) intr[INTR_STOP] = 1'b1;
            if (sp_w_cnt == 1 || sp_w_cnt == 2) begin
              stack[sp_min-1] = new_top;
            end
            if (sp_w_cnt == 2) begin
              stack[sp_min-2] = new_btop;
            end
            if (pmem_we) begin
              state <= str;
            end else if (slp || stop || single_step) begin
              state <= slp;
            end else begin
              state <= fetch;
            end
          end
          str: begin
            // Store data from instruction into either code or data memory
            mem_select   <= 1;
            mem_write_en <= 1;
            if (mem_data_ready) begin
              mem_select <= 0;
              mem_write_en <= 0;
              state <= single_step ? slp : fetch;
            end
          end
          slp: begin
            // The only way to leave this state is via CPU intervention.
          end
          default: state <= 3'bx;
        endcase
      end
    end
  end

`ifdef FORMAL
  reg f_init = 1;
  always @(posedge clk) begin
    if (f_init) assume (reset);
    if (!reset) begin
      assert (!slp || !stop);
      assert(
        state == fetch ||
        state == ex ||
        state == str ||
        state == data
        state == slp
      );
      if (i_wb_stb && i_wb_cyc && $past(i_wb_stb) && $past(i_wb_cyc)) begin
        assume ($past(i_wb_addr) == i_wb_addr);
        assume ($past(i_wb_data) == i_wb_data);
        assume ($past(i_wb_we) == i_wb_we);
      end
      if (state != fetch && state != data && state != str) begin
        assert (!mem_select);
      end
      if (state != str) begin
        assert (!mem_write_en);
      end
    end
    f_init <= 0;
  end
`endif  /* FORMAL */

endmodule
