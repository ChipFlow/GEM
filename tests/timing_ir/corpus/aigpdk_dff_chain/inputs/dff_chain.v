// Minimal AIGPDK design exercising all four IR record types:
// - ARC          (DFF CLK->Q, AND A/B->Y)
// - SETUP_HOLD   (DFF D vs CLK)
// - CLOCK_ARRIVAL (DFF clock pin, with the SDC below)
// - INTERCONNECT (back-annotated wire between DFF.Q and AND.A from the SDF)
//
// Used by the WS4 corpus regression test
// (tests/timing_ir/corpus/README.md). Keep it small — corpus regen is
// run on every OpenSTA pin bump and a larger design pays for itself only
// when it covers a record type the small entries don't.

module dff_chain(CLK, D, Q);
  input CLK;
  input D;
  output Q;
  wire mid;
  DFF d1 (.CLK(CLK), .D(D), .Q(mid));
  AND2_00_0 a1 (.A(mid), .B(D), .Y(Q));
endmodule
