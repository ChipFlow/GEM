// CVC functional (no SDF) testbench for MCU SoC — diagnostic.
// Same as tb_cvc.v but without SDF annotation.

`timescale 1ps/1ps

module tb_cvc;

  reg por_l;
  reg porb_h;
  reg porb_l;
  reg resetb_h;
  reg resetb_l;
  reg [43:0] gpio_in;
  reg [43:0] gpio_in_h;
  reg [43:0] gpio_loopback_one;
  reg [43:0] gpio_loopback_zero;
  reg [31:0] mask_rev;

  wire [43:0] gpio_out;
  wire [43:0] gpio_oeb;
  wire [43:0] analog_io;
  wire [43:0] analog_noesd_io;

  openframe_project_wrapper uut (
    .por_l(por_l),
    .porb_h(porb_h),
    .porb_l(porb_l),
    .resetb_h(resetb_h),
    .resetb_l(resetb_l),
    .analog_io(analog_io),
    .analog_noesd_io(analog_noesd_io),
    .gpio_in(gpio_in),
    .gpio_in_h(gpio_in_h),
    .gpio_loopback_one(gpio_loopback_one),
    .gpio_loopback_zero(gpio_loopback_zero),
    .gpio_oeb(gpio_oeb),
    .gpio_out(gpio_out),
    .mask_rev(mask_rev)
  );

  // No SDF annotation — pure functional simulation

  initial begin
    $dumpfile("cvc_output.vcd");
    $dumpvars(0, gpio_out);
    $dumpvars(0, gpio_oeb);
    $dumpvars(0, gpio_in);
  end

  initial begin
    por_l = 1'b1;
    porb_h = 1'b1;
    porb_l = 1'b1;
    resetb_h = 1'b1;
    resetb_l = 1'b1;
    gpio_in = 44'h0;
    gpio_in_h = 44'h0;
    gpio_loopback_one = 44'h0;
    gpio_loopback_zero = 44'h0;
    mask_rev = 32'h0;
  end

  `include "stimulus_gen.v"

endmodule
