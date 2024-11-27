/*
 * Copyright (c) 2024 Chinmay Dhole
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_shiftreg (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  // All output pins must be assigned. If not used, assign to 0.
  //assign uo_out  = 0;  // Example: ou_out is the sum of ui_in and uio_in
  assign uio_out = 0;
  assign uio_oe  = 0;

  // List all unused inputs to prevent warnings
    wire _unused = &{uio_in, 1'b0};
   shiftreg sr(
    .clk(clk),
    .rst(rst_n),
    .shift_enable(ena),
    .data_in(ui_in),
    .data_out(uo_out)
    );
  endmodule
  module shiftreg (
    input clk,
    input rst,
    input shift_enable,
    input [7:0] data_in,
    output [7:0] data_out
    );
    parameter N = 1000;  // Number of registers
    reg [7:0] reg_array [0:N-1];
    generate
    genvar i;
    for (i = 0; i < N; i = i + 1) begin : gen_reset
        always @(posedge clk or posedge rst) begin
            if (rst) begin
                reg_array[i] <= 8'd0;
            end else if (shift_enable) begin
                if (i == 0)
                    reg_array[0] <= data_in;
                else
                    reg_array[i] <= reg_array[i-1];
            end
        end
    end
endgenerate

  assign data_out = reg_array[N-1];
   
endmodule


