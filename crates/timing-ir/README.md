# `timing-ir`

Jacquard's timing intermediate representation — SDF-equivalent timing annotation data in a schema-versioned, zero-copy FlatBuffers format.

Design rationale: [`docs/adr/0002-timing-ir.md`](../../docs/adr/0002-timing-ir.md).
Requirements: [`docs/timing-correctness.md`](../../docs/timing-correctness.md) R1.

## Scope

This crate defines and implements read/write for the timing IR. It does **not** parse SDF, Liberty, SPEF, or Verilog; those are converter crates (`opensta-to-ir`, future native Rust converters). It does not represent netlists, timing graphs, or cell characterisation.

## Layout

```
crates/timing-ir/
├── Cargo.toml
├── README.md                   # this file
├── schemas/
│   └── timing_ir.fbs           # source of truth — edit here first
└── src/
    ├── lib.rs                  # crate root, re-exports
    └── schema_generated.rs     # flatc-generated code (checked in — see below)
```

## Dependencies

External (install before building or regenerating bindings):

| Dependency | Version | macOS install | Linux (Debian/Ubuntu) install |
|---|---|---|---|
| `flatc` (FlatBuffers compiler) | 25.x (tested: 25.12.19) | `brew install flatbuffers` | `sudo apt install flatbuffers-compiler` |

Rust dependencies are declared in `Cargo.toml`; `flatbuffers = "25"` is the runtime matching the `flatc` major version above.

If you hit a version-mismatch error between `flatc`-generated code and the `flatbuffers` runtime crate, bump both together. The project does not support mixing majors.

## FlatBuffers toolchain

**Generator**: the official `flatc` compiler from [`google/flatbuffers`](https://github.com/google/flatbuffers).
**Runtime**: the [`flatbuffers`](https://crates.io/crates/flatbuffers) Rust crate.

The generated Rust bindings (`src/timing_ir_generated.rs`) are **checked into the repository**. Rationale: during Phase 0 the schema will iterate; checking in the generated code makes the schema-and-bindings pair reviewable together, and avoids forcing every contributor and CI runner to install `flatc` up front to build Jacquard. Only contributors modifying the schema need `flatc` installed. Phase 1 may replace this with `build.rs` codegen once the schema settles.

### Regenerating bindings

Install `flatc` first (see Dependencies above), then:

```bash
cd crates/timing-ir
flatc --rust -o src/ schemas/timing_ir.fbs
cargo fmt                    # rustfmt touches the generated file; keep it clean
cargo test                   # verify the round-trip still passes
```

`flatc` writes `src/timing_ir_generated.rs`; the crate's `lib.rs` includes it via `#[path]`. Commit the schema change and the regenerated file together in the same PR.

## JSON round-trip

`flatc` can emit a JSON representation of any IR binary, and can read JSON back and emit binary. This is used in CI for human-readable diffs (WS4 in the phase-0 plan).

```bash
# Binary → JSON
flatc --json --strict-json --raw-binary \
    --root-type jacquard.timing_ir.TimingIR \
    -o /tmp/ir-json/ \
    schemas/timing_ir.fbs \
    -- my_design.jtir

# JSON → Binary
flatc --binary \
    --root-type jacquard.timing_ir.TimingIR \
    schemas/timing_ir.fbs \
    my_design.json
```

## License

Apache-2.0, matching the parent Jacquard project.
