# Timing-model extensions — design notes

**Status:** Idea / pre-spike. Not scheduled. Captured here so the architecture sketch survives the next session-clear.

**Scope:** Three related extensions to Jacquard's timing model, all aimed at making setup/hold reporting more honest without abandoning the cycle-accurate boomerang kernel.

1. **Dynamic delay** — per-gate δ(T) inspired by the Involution Delay Model (Maier 2021, [arXiv:2107.06814](https://arxiv.org/abs/2107.06814)). Captures pulse-width-dependent delay degradation that fixed δ∞ misses on near-threshold paths.
2. **Clock-tree skew** — per-DFF clock arrival accounting. Today every DFF on a clock is treated as if it captures simultaneously; SDF clock-buffer arcs and clock-net interconnect are silently dropped during AIG construction.
3. **Wire delay at scale** — per-receiver interconnect delay applied to the right edge in the AIG, and explicit modelling of inter-partition wires. Today wire delay is collapsed to a max-per-destination-cell scalar — fine for sky130 short routes, increasingly wrong as we move to faster clocks, finer processes, and large many-core/NoC designs.

All three share the same insight: the data the model needs is already in the TimingIR. The work is at the consumer layer (`flatten.rs`, `aig.rs`, the kernel arrival math), not the IR or the partitioner.

---

## Background — what the timing pipeline does today

```
.sdf  ─┬─► opensta-to-ir ──► TimingIR (.jtir, FlatBuffers)
.jtir ─┘                          │
                                  ▼
              flatten.rs::load_timing_from_ir   (per-cell arc → AIG-pin delay)
                                  │
                                  ▼
                       gate_delays: Vec<PackedDelay>     (rise/fall ps per AIG pin)
                       dff_constraints: Vec<DFFConstraint>  (setup/hold ps per DFF)
                                  │
                                  ▼
              flatten.rs::inject_timing_to_script   (bake max ps into u16 script slot)
                                  │
                                  ▼
                       kernel_v1.metal at runtime:
                       per-AND:    new_arr = max(arr_a, arr_b) + gate_delay
                       per-DFF:    check arrival vs setup/hold per word
```

Reference points:
- IR schema: `crates/timing-ir/schemas/timing_ir.fbs`
- IR consumer: `src/flatten.rs:1768` (`load_timing_from_ir`), `src/flatten.rs:1686` (`inject_timing_to_script`)
- Setup/hold buffer: `src/flatten.rs:1732` (`build_timing_constraint_buffer`)
- GPU arrival math: `csrc/kernel_v1.metal:220-255` (AND gates), `csrc/kernel_v1.metal:547-580` (setup/hold)

Per-AIG-pin arrival is a single `ushort` accumulated by `max` through the boomerang reduction. There is no event scheduling — arrival is a scalar that rides alongside the Boolean evaluation in lockstep with cycle ticks.

---

## Part A — Dynamic delay (IDM-style δ(T))

### What IDM is, briefly

A per-gate dynamic delay model that makes δ a function of `T` (time since the gate's last output transition). The distinguishing property: input pulses with Δᵢ → 0 have diminishing effect on the output. The model handles pulse-width degradation faithfully and is the only model proven to solve the short-pulse-filtration problem. The paper notes ~80–590% CPU overhead vs. inertial delay on a CPU event-driven simulator.

### The architectural wall

True IDM needs event scheduling and intra-cycle pulse observability — neither is available in Jacquard's lockstep cycle-accurate kernel. We **cannot** model glitch suppression or metastability oscillation traces without either sub-cycle ticks or a different kernel architecture.

What we **can** do is enrich the per-gate delay used in arrival propagation so setup/hold reporting reflects realistic pulse degradation on marginal paths.

### Five hook points

| Hook | File | Today | With δ(T) |
|---|---|---|---|
| **A** Schema | `crates/timing-ir/schemas/timing_ir.fbs` | rise/fall per arc | + per-cell-type `DynamicDelayParams` (exp-channel params or piecewise-linear LUT) |
| **B** IR load | `src/flatten.rs:1768` | one `PackedDelay` per AIG pin | + parallel `gate_dyn_delays` keyed by originating cell-type via `aigpin_cell_origins` |
| **C** Bake | `src/flatten.rs:1686` | one u16 ps per thread slot | static-IDM: bake worst-case δ(T) into same slot. dynamic-IDM: reserve second u32 |
| **D** Kernel arrival | `csrc/kernel_v1.metal:220-255` | `max(arr_a, arr_b) + gate_delay` | `+ eval_idm(dyn_params, T, edge)` via small LUT |
| **E** Setup/hold | `csrc/kernel_v1.metal:547-580` | unchanged math, dumber inputs | unchanged math, smarter inputs |

For dynamic-IDM the kernel needs two new persistent buffers:
- `last_transition_ps[aig_pin]` — when the gate's output last switched (absolute ps).
- `last_value[aig_pin]` — to detect transitions across cycles.

Memory cost ~4 bytes per AIG pin per partition. For NVDLA-scale designs (~hundreds of thousands of pins) this is MB-scale — fine.

### `eval_idm` on GPU

The paper uses exp/log per gate. On GPU replace with a 16-entry LUT indexed by quantised `T`. Cheap, branch-free, smooth enough.

### Characterisation

The δ(T) parameters have to come from per-cell SPICE characterisation. For sky130 we'd characterise each `sky130_fd_sc_hd__*_*` cell once, check the result into the repo, and ship it as a sidecar table consumed by the IR builder. This is the expensive one-off — the paper flags characterisation cost as the unsolved part of making IDM "truly competitive."

### Staged plan

| Stage | What | Touches | Kernel | Effort | Win |
|---|---|---|---|---|---|
| **1** Static IDM | Bake worst-case δ(T) into existing u16 slot using STA pulse-width estimates | A, B, C | None | 1–2 days | Better setup/hold on marginal paths |
| **2** Dynamic δ(T) | Add `last_transition_ps` buffer + LUT eval | All | Lines 220–255 | 1–2 weeks | Pulse-degradation-aware arrivals end-to-end |
| **3** Sub-cycle ticks | Multiple arrival propagations per logical cycle | Whole kernel | Major | Months | True IDM glitch behaviour. **Probably not worth it** for Jacquard's positioning. |

Stage 1 is a 1–2 day spike with no kernel risk. Stage 2 is the honest implementation. Stage 3 is a different simulator.

### What we get / don't get from dynamic δ(T)

**Achievable**
- Per-corner δ(T) propagating through arrival → setup/hold reports that distinguish "just meets timing under δ∞" from "fails under realistic pulse degradation".
- Stays inside cycle-accurate boomerang. ~1.5–2× memory growth on arrival data, ~10–20% kernel slowdown (estimate).

**Not achievable**
- Glitch suppression (Δᵢ → 0 → no transition).
- Metastable oscillation traces.
- Combinational-loop behaviours (loops are forbidden in the AIG anyway).

### Why sky130 is the right vehicle

`sky130_pdk.rs` decomposes vendor functional Verilog into AIG nodes while preserving cell identity through `aigpin_cell_origins`. We can attach δ(T) at the **original sky130 cell** granularity even after AIG flattening — that structural property is what makes any of this tractable. Cells from a hand-coded library without origin tracking would be much harder.

---

## Part B — Clock-tree skew

### Where the information is — and where we drop it

Clocks in Jacquard are walked back from each DFF through buffers/inverters/clock-gates, terminating at an `InputClockFlag(pinid, is_negedge)` (`src/aig.rs:441`, `:477`, `:495-560`). Recognised cells: `INV/BUF/CKLNQD` and the sky130 equivalents `inv*`, `clkinv*`, `buf*`, `clkbuf*`, `clkdlybuf*`, `lpflow_*`.

Two consequences:

1. **Clock-tree cells produce no AIG pin.** They collapse into a polarity flag on the DFF. Since `aigpin_cell_origins` only lists cells that produced AIG pins, the timing-IR arcs on those cells (`IOPATH` records on `clkbuf_8`, etc.) match no AIG pin in `load_timing_from_ir` and are silently discarded.
2. **Clock-net interconnect is dropped the same way.** `interconnect_delays` records keyed by net endpoints have no destination cell to attach to, so they fall on the floor.

Net effect: every DFF on a given clock domain is treated as having **identical clock arrival**, i.e. perfect skew. The current setup/hold check is honest about combinational-path delay but blind to clock-tree topology.

For a sky130 MCU SoC at ~25 ns clock period this is fine functionally; for any timing claim near the period boundary it's misleading. Intra-domain clock-tree skew on sky130 is typically O(50–200 ps) — small relative to a 25 ns period, but exactly the order of magnitude that determines whether a path "barely meets" or "barely fails" setup.

### Do we have the information?

Yes, in three places, in increasing fidelity:

1. **TimingIR arcs on clock cells** (`.jtir` already contains them; we just don't consume them).
2. **The AIG clock walk** in `aig.rs:495–560` already iterates the clock-side cells of each DFF in order. It just doesn't accumulate their delays. Adding a `dff_clock_origins: Vec<Vec<cellid>>` parallel structure costs O(num_dffs × clock_depth) memory — negligible.
3. **OpenTimer / OpenSTA** can compute per-DFF clock arrival end-to-end, including common-path-pessimism removal (CRPR), which manual accumulation misses. ADR 0003 already nominates OpenTimer as primary STA. The cleanest version of this is a new IR table:

   ```fbs
   table ClockArrival {
       cell_instance: string;     // DFF instance path
       clk_pin: string;           // local pin name
       arrival: [TimingValue];    // per-corner clock arrival ps
       provenance: Provenance;
   }
   ```

   Populated by `opensta-to-ir` (or whichever STA we standardise on) during IR generation. Consumer code never touches the netlist — it just looks up each DFF's clock arrival.

### Consumer change

`DFFConstraint` (`src/flatten.rs:75-86`) today:

```rust
pub struct DFFConstraint {
    pub setup_ps: u16,
    pub hold_ps: u16,
    pub data_state_pos: u32,
    pub cell_id: u32,
}
```

Add `clock_arrival_ps: i16` (signed — capture-side arrival relative to a reference). The setup/hold check in `build_timing_constraint_buffer` (`src/flatten.rs:1732`) becomes the standard inter-DFF skew formula:

- **Setup margin** = `(clock_period + clock_arr_capture - clock_arr_launch) - data_arrival - setup`
- **Hold margin** = `data_arrival - (clock_arr_capture - clock_arr_launch + hold)`

Per-launch/per-capture pairing is awkward in the current per-word-collapsed constraint buffer; the simplest first step is to **fold the capture-side clock arrival into the per-DFF setup/hold values themselves** before packing:

- effective_setup = `setup - clock_arrival_capture` (so positive arrival eats setup margin)
- effective_hold = `hold + clock_arrival_capture`

Then the existing GPU check works unchanged. This treats launch arrival as zero (ref) — pessimistic for paths that span domains with very different launch arrivals, but correct as a first cut.

### Partitioning question

> "could we partition a design effectively to do this somewhat accurately without sacrificing too much?"

Today partitioning (`src/repcut.rs`) is hypergraph-cut on logic connectivity. DFFs co-located by logic affinity may have very different clock arrivals.

The pessimism cost: `build_timing_constraint_buffer` collapses all DFFs in a 32-bit state word to `min(setup)` and `min(hold)`. If a word holds DFFs with clock arrival 50 ps and 200 ps, the per-word effective setup is the worst of both — i.e. we report timing as if every DFF in that word saw the worst skew in the word. That's a 150 ps pessimism for the lucky DFF.

Three options, ranked:

1. **Do nothing.** For typical sky130 SoCs at ≥10 ns clock periods, intra-word skew (≤200 ps worst-case) vs. period (10 000+ ps) is ≤2%. Worth-it threshold for the optimisation: when designs run close enough to the period that 2% pessimism flips genuine passes into reported violations. Likely never for sky130. Plausibly relevant for designs running at ≥1 GHz on a more aggressive PDK.

2. **Skew-bucket the DFF constraint packing, not the partitioning.** Group DFFs into clock-arrival buckets *after* partitioning, and emit one constraint word per bucket-within-partition rather than collapsing everything in the word. Increases constraint-buffer size by O(num_buckets) but doesn't disturb the partitioner. **Probably the right answer if we ever need to.**

3. **Skew-aware partitioning.** Add a soft objective to `repcut.rs` that prefers grouping DFFs by clock arrival. Degrades cut quality (more inter-partition logic edges → more state shuffling). Almost certainly worse than option 2 for the same accuracy gain.

So: yes we have the info, no we probably don't need to repartition, and the constraint-collapsing pessimism is the real lever — either accept it (option 1) or break it bucket-wise (option 2).

### Staged plan for clock tree

| Stage | What | Touches | Kernel | Effort |
|---|---|---|---|---|
| **1** Capture clock-tree delay | Add `ClockArrival` IR table; populate from opensta-to-ir | IR schema, `opensta-to-ir/builder` | None | 2–3 days |
| **2** Apply to setup/hold | Fold capture-side arrival into `DFFConstraint`; existing kernel check now skew-aware | `src/flatten.rs:75-86`, `:1732` | None | 1–2 days |
| **3** (if needed) Bucketed packing | Per-bucket constraint words; kernel reads the right bucket per DFF | `src/flatten.rs:1722-1761`, kernel constraint indexing | Minor | 3–5 days |

Stage 1+2 covers the realistic accuracy win. Stage 3 only if measurement shows the per-word collapse is materially over-reporting violations.

---

## Part C — Wire delay at scale

### Why this gets more important as designs grow

In sky130 at 25 ns clock periods, wire delay is a small perturbation on gate delay and the lumped model is fine. The picture changes in two regimes:

- **Faster clocks.** Wire delay is a fixed physical quantity (RC-dominated); period shrinks; wire fraction of the budget grows.
- **Finer processes (e.g. 22nm and below).** Gate delays scale down with feature size; wire RC scales **unfavourably** (resistance per square goes up, capacitance per length stays roughly flat). The classic "reverse scaling" inflection: gates get faster, long wires don't. Typical 22nm: inverter delay 5–15 ps, local short wires 5–20 ps, global routes 50–500 ps, multi-mm wires 1+ ns without repeaters.
- **Large many-core/NoC SoCs.** Inter-tile mesh links can span multiple millimetres; chip-level signals have wire delays comparable to or larger than entire combinational stages.

For a many-small-core NoC at 22nm, wire delay on inter-core links is typically **the** dominant timing factor. Any model that can't represent it accurately will misreport the critical paths.

### What Jacquard does today

The IR side is already in shape. `crates/timing-ir/schemas/timing_ir.fbs` carries `InterconnectDelay { net, from_pin, to_pin, delay[corner] }` per receiver, and `opensta-to-ir` populates it from SDF.

The lossy step is the **consumer** in `src/flatten.rs:1850-1872`:

```rust
let mut wire_delays_per_cell: HashMap<usize, (u64, u64)> = HashMap::new();
// ... for each InterconnectDelay record:
let entry = wire_delays_per_cell.entry(dest_cellid).or_insert((0, 0));
entry.0 = entry.0.max(d);   // rise
entry.1 = entry.1.max(d);   // fall (same value!)
```

Three layers of pessimism stacked here:

1. **Keyed by destination cell, not destination pin.** A cell with two inputs from very different routes loses per-pin fidelity.
2. **Max across inputs of the same cell.** Worst-case incoming wire is applied to every output of the cell.
3. **No rise/fall distinction on wire delay.** SDF carries both; we collapse to one number.

Then in arrival propagation (`csrc/kernel_v1.metal:220-255`):

```c
new_arr = max(arr_a, arr_b) + gate_delay
```

where `gate_delay = intrinsic + max_wire_into_cell`. The mathematically correct propagation is:

```c
new_arr = max(arr_a + wire_a, arr_b + wire_b) + intrinsic
```

These are equivalent **only when the input with the worst arrival also has the worst wire**. When they don't coincide — common on a NoC node where one input comes from a long mesh hop and another from local logic — the current model over-reports by `max_wire − actual_wire_on_critical_input`.

For sky130 small designs this gap is in the noise. For 22nm with 10× variation between local and global wire delays, it's the difference between "this path meets timing" and "STA reports a violation that doesn't exist."

### Inter-partition wires — the architectural wrinkle

A NoC tile naturally maps to one (or a few) partition(s). The inter-tile links — the long, wire-dominated, timing-critical ones — are precisely the partition-crossing signals. Today wire delay sits on the destination cell's `gate_delays` slot, evaluated inside the destination partition's boomerang reduction. The wire is a property of the **crossing**, not the destination cell, and should ideally be modelled at the partition I/O boundary, where `src/sim/cosim_metal.rs` already shuffles state between partitions.

This is the inverse alignment of the clock-tree case. There partitioning didn't help with skew accounting. Here partitioning is *load-bearing*: tile-aligned partitions naturally expose the small set of edges that deserve careful wire-delay modelling, and let intra-partition logic stay on the fast lumped path.

### Three fidelity tiers

| Tier | Model | Where wire delay lives | When it's enough |
|---|---|---|---|
| **0** (current) | One scalar per destination cell, max-collapsed | Folded into `gate_delays[output_pin]` of dest cell | sky130 + ≥10 ns periods + small designs |
| **1** Per-receiver | One scalar per `(from_pin, to_pin)` edge in the AIG | Folded into the **source** AIG pin's gate_delay, with one entry per fanout target | Local wires in faster designs; intra-tile NoC logic |
| **2** Per-edge with inter-partition arcs | Tier 1 + explicit wire delay on partition-crossing signals | Tier 1 + new arrival-bump applied during `cosim_metal.rs` state shuffle | Long routes + many-core/NoC + 22nm-scale processes |

Tier 1 is mostly a `flatten.rs` rewrite. Tier 2 needs `cosim_metal.rs` extension and a new field in the inter-partition transfer format.

### Information availability

Yes, it's there:
- `InterconnectDelay` records exist per receiver. SDF carries them. opensta-to-ir emits them.
- Per-input-pin granularity is in the IR (`to_pin` includes the local pin name). The consumer just discards it via `to_pin.rfind('/')` to derive `dest_inst`.
- Rise/fall distinction is in the schema (`delay: [TimingValue]` per corner; rise/fall could be on top via the same pattern as `TimingArc`). For SDF-back-annotated flows the rise/fall split usually comes from the SDF; we'd need to confirm opensta-to-ir preserves both edges.

What's missing today:
- Tier-1 plumbing: AIG-pin-level wire delay per fanout. Current `gate_delays: Vec<PackedDelay>` is keyed by AIG pin (the output side); to do per-input-edge correctly we want delay attached to the *edge*, not the node. Either add a parallel `wire_delays: HashMap<(src_aigpin, dst_aigpin), PackedDelay>` or refactor toward an edge-attributed AIG.
- Tier-2 plumbing: a "partition-crossing arc" concept in `cosim_metal.rs`. Currently inter-partition state shuffle moves bits with no associated arrival bump. Adding a per-edge ps adjustment is straightforward in principle; finding the right place in the shuffle pipeline matters.

### IR scale

The IR-size concern bites here. `InterconnectDelay` is roughly 100–200 bytes per record; a 22nm SoC with 10⁶–10⁷ nets is a .jtir file in the hundreds-of-MB to multi-GB range.

Mitigations:
- Streaming load: today `TimingIrFile::from_path` reads the whole buffer. Could mmap and lazy-decode, since FlatBuffers is offset-based.
- Sharding: split IR per partition or per top-level module. Adds a build-time step but bounds memory per process.
- Drop intra-cell wires from IR generation: SDF often has microscopic interconnect records that lump into the destination's own pin-cap. Filter these out at the opensta-to-ir builder. Loss is genuinely negligible.

Worth measuring before committing to mitigations — sky130 NVDLA-scale today is fine; the question is what 22nm + N-tile mesh looks like.

### Partitioning question — the other direction

For NoC designs partitioning becomes a positive lever (unlike the clock-tree case where it was neutral). Two specific levers:

1. **Tile-aligned partitions.** If `repcut.rs` finds tile-aligned cuts naturally (likely, given typical tile-to-tile connectivity sparsity), inter-partition arcs are a small, well-defined set of NoC links. Worth verifying with a representative design — a partitioning report keyed by signal name pattern (`*_link_*`, `noc_*`, configurable) would expose whether the partitioner's logic-affinity score is already aligned with tile boundaries or whether we need to bias it.
2. **NoC-link partitioning hint.** Add a soft bias to repcut that prefers cutting nets matching a configured regex. Same partitioning machinery, configurable input. Cost: degrades cut quality if the hint conflicts with logic affinity. Likely worth it for explicitly tile-decomposed designs where the user knows the tile boundaries; not worth it for flat designs.

The point of any of this is to make Tier-2 cheap: if the inter-partition arc set is small, per-edge wire delay on those crossings costs almost nothing.

### Crosstalk and OCV

These are upstream concerns. SDF from a crosstalk-aware STA flow already carries pessimistic delays; OCV (on-chip variation) is similarly baked into the chosen corner. Jacquard consumes whatever the IR was generated against. Worth a one-line note in the user-facing docs that the timing report's accuracy is bounded by the SDF/STA flow it was built from — Jacquard does not invent crosstalk pessimism.

### Staged plan for wire delay

| Stage | What | Touches | Kernel | Effort |
|---|---|---|---|---|
| **1** Per-receiver consumption | Key wire delay by `(src_aigpin, dst_aigpin)` edge; fold into source AIG pin's gate_delay per fanout | `src/flatten.rs:1850-1872`, possibly `src/aig.rs` for fanout tracking | None | 3–5 days |
| **2** Rise/fall distinction | Preserve per-edge rise/fall through the consumer; honour both in `PackedDelay` accumulation | `src/flatten.rs:1850-1914` | None | 1–2 days |
| **3** Inter-partition arc delay | New per-crossing wire-delay table; arrival bump applied during inter-partition state transfer | `src/sim/cosim_metal.rs` shuffle path; `src/flatten.rs` partition-boundary metadata | Yes (transfer path) | 2–3 weeks |
| **4** IR scale plumbing | Streaming/mmap load; opensta-to-ir filtering of microscopic records | `src/sim/timing_ir_loader.rs`, `opensta-to-ir/builder` | None | 1 week (gated on measurement) |
| **5** NoC-aware partitioning | Soft bias in repcut for cutting flagged nets; partition report by tile | `src/repcut.rs` and CLI flags | None | 1–2 weeks |

For a sky130 use case Stage 1+2 likely covers everything you'd notice. For 22nm NoC, Stages 1–3 are the meaningful set; Stage 5 is the optimisation that makes Stage 3 cheap.

### What we get / don't get

**Achievable**
- Setup/hold accuracy on long routes that today gets clobbered by max-collapse pessimism.
- Honest reporting on NoC inter-tile links — the paths that actually matter for many-core SoC timing closure.
- All of the above without changing Jacquard's cycle-accurate kernel architecture.

**Not achievable from this work alone**
- Crosstalk-driven delay uncertainty (handled upstream in STA).
- Variation-aware (statistical) timing — would need OCV-corner sweeping or SSTA, neither of which is on the roadmap.
- Process variation modelling beyond the corners the SDF/IR was generated against.

---

## Open questions

1. **δ(T) characterisation cost.** One-off SPICE per cell-type per corner. Cheaper if we lean on existing ECSM/CCSM data already in vendor Liberty rather than re-running SPICE. Worth investigating before committing to Stage 2.
2. **Whose clock arrival is authoritative?** Manual accumulation in `aig.rs` is simple but misses CRPR. STA-tool arrivals (OpenTimer) are correct but couple us to that tool's output format. ADR 0003 leans toward OpenTimer; reuse that decision.
3. **Interaction.** Does δ(T) on clock-tree buffers matter? Probably not enough to model — clock buffers are sized for fast edges and operate far from their pulse-degradation regime. But the framework should be able to express "ignore δ(T) on clock domain" cleanly.
4. **Validation oracle.** CVC and Icarus already serve as functional oracles; for skew-aware and wire-aware reporting we'd want OpenTimer's slack report as the ground truth for unit tests. ADR 0003 again.
5. **IR size at 22nm scale.** Open question whether `.jtir` for a representative many-core NoC fits in available memory under the current eager-load model. Needs measurement before committing to streaming mitigations.
6. **Edge-attributed AIG.** Per-receiver wire delay wants delay attached to AIG *edges*, not nodes. Today the AIG is node-attributed (`gate_delays: Vec<PackedDelay>` indexed by aigpin). A clean Tier-1 implementation may push toward edge attribution, with downstream effects on the boomerang reduction script layout. Worth a small spike before the main implementation.
7. **Partition-crossing format.** Adding per-edge wire delay to `cosim_metal.rs` inter-partition transfers needs a precise place in the existing pipeline. Currently the shuffle moves Boolean state words without arrival; the natural place is alongside the writeout-arrival path that already exists for setup/hold checking, but the alignment isn't 1:1 because partition crossings happen at logic boundaries, not capture-DFF boundaries.

## Related artefacts

- `docs/timing-correctness.md` — forward-looking validation contract; this doc extends rather than replaces.
- `docs/timing-simulation.md` — boomerang architecture; the kernel-side context.
- `docs/timing-validation.md` — current ±5% acceptance criteria; would tighten under δ(T).
- `docs/adr/0002-timing-ir.md` — IR design rationale; schema additions here follow the "lossless extension" principle.
- `docs/adr/0003-opentimer-primary-sta.md` — STA tool decision; relevant to clock-arrival sourcing.
