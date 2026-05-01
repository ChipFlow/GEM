//! Dump format parser.
//!
//! The OpenSTA Tcl driver emits a line-oriented, tab-separated record format
//! consumed by this parser. The format is a private contract between the
//! Tcl driver and this binary — both ship together, both bump
//! `format-version` together. See `docs/plans/ws2-opensta-to-ir.md` for the
//! record layout.
//!
//! Phase 0 WS2 (Phase 2.1) populates only `CORNER` and `ARC` records.
//! `INTERCONNECT`, `SETUP_HOLD`, and `VENDOR_EXT` are parsed structurally
//! so future phases can wire them up without changing the parser.

use std::str::FromStr;

/// The format version this parser understands. Bumped together with the
/// Tcl driver when the dump shape changes incompatibly.
pub const FORMAT_VERSION: u32 = 1;

/// Origin label as emitted by the Tcl driver. Mirrors `timing_ir.fbs`'s
/// `Origin` enum for direct mapping during IR build.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Origin {
    Asserted,
    Computed,
    Defaulted,
}

impl FromStr for Origin {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "Asserted" => Ok(Origin::Asserted),
            "Computed" => Ok(Origin::Computed),
            "Defaulted" => Ok(Origin::Defaulted),
            other => Err(format!("unknown origin {other:?}")),
        }
    }
}

/// Edge label as emitted by the Tcl driver.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Edge {
    Posedge,
    Negedge,
}

impl FromStr for Edge {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "Posedge" => Ok(Edge::Posedge),
            "Negedge" => Ok(Edge::Negedge),
            other => Err(format!("unknown edge {other:?}")),
        }
    }
}

#[derive(Clone, Debug)]
pub struct CornerRecord {
    pub index: u32,
    pub name: String,
    pub process: String,
    pub voltage: f64,
    pub temperature: f64,
}

#[derive(Clone, Debug)]
pub struct ArcRecord {
    pub cell_instance: String,
    pub driver_pin: String,
    pub load_pin: String,
    pub corner_index: u32,
    pub rise_min: f64,
    pub rise_typ: f64,
    pub rise_max: f64,
    pub fall_min: f64,
    pub fall_typ: f64,
    pub fall_max: f64,
    pub condition: String,
    pub origin: Origin,
}

#[derive(Clone, Debug)]
pub struct InterconnectRecord {
    pub net: String,
    pub from_pin: String,
    pub to_pin: String,
    pub corner_index: u32,
    pub min: f64,
    pub typ: f64,
    pub max: f64,
    pub origin: Origin,
}

#[derive(Clone, Debug)]
pub struct SetupHoldRecord {
    pub cell_instance: String,
    pub d_pin: String,
    pub clk_pin: String,
    pub edge: Edge,
    pub corner_index: u32,
    pub setup_min: f64,
    pub setup_typ: f64,
    pub setup_max: f64,
    pub hold_min: f64,
    pub hold_typ: f64,
    pub hold_max: f64,
    pub condition: String,
    pub origin: Origin,
}

#[derive(Clone, Debug)]
pub struct ClockArrivalRecord {
    pub cell_instance: String,
    pub clk_pin: String,
    pub corner_index: u32,
    pub min: f64,
    pub typ: f64,
    pub max: f64,
    pub origin: Origin,
}

#[derive(Clone, Debug)]
pub struct VendorExtRecord {
    pub source: String,
    pub source_tool: String,
    pub kind: String,
    pub raw_payload_b64: String,
}

#[derive(Clone, Debug)]
pub enum DumpRecord {
    Corner(CornerRecord),
    Arc(ArcRecord),
    Interconnect(InterconnectRecord),
    SetupHold(SetupHoldRecord),
    ClockArrival(ClockArrivalRecord),
    VendorExt(VendorExtRecord),
}

/// Header metadata extracted from the `# key: value` preamble.
#[derive(Clone, Debug, Default)]
pub struct DumpHeader {
    pub format_version: u32,
    pub generator_tool: String,
    pub generator_opensta: String,
    pub input_files: Vec<String>,
}

/// A successfully parsed dump document.
#[derive(Clone, Debug)]
pub struct DumpDocument {
    pub header: DumpHeader,
    pub records: Vec<DumpRecord>,
}

/// Parse error with a 1-indexed line number for diagnostics.
#[derive(Clone, Debug)]
pub struct ParseError {
    pub line: usize,
    pub message: String,
}

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "line {}: {}", self.line, self.message)
    }
}

impl std::error::Error for ParseError {}

/// Parse the entire dump document from a string. The parser is strict —
/// unknown record types or malformed fields produce a fail-loud error.
pub fn parse_dump(content: &str) -> Result<DumpDocument, ParseError> {
    let mut header = DumpHeader::default();
    let mut records = Vec::new();
    let mut saw_version = false;
    let mut saw_end = false;

    for (idx, raw_line) in content.lines().enumerate() {
        let lineno = idx + 1;
        let line = raw_line.trim_end_matches('\r');
        if line.is_empty() {
            continue;
        }
        if let Some(stripped) = line.strip_prefix('#') {
            let trimmed = stripped.trim_start();
            if trimmed == "end" {
                saw_end = true;
                continue;
            }
            parse_header_line(trimmed, &mut header, &mut saw_version, lineno)?;
            continue;
        }

        let mut fields = line.split('\t');
        let kind = fields
            .next()
            .ok_or_else(|| err(lineno, "empty record line"))?;
        let rest: Vec<&str> = fields.collect();
        let record = parse_record(kind, &rest, lineno)?;
        records.push(record);
    }

    if !saw_version {
        return Err(err(0, "missing `# format-version` header line"));
    }
    if header.format_version != FORMAT_VERSION {
        return Err(err(
            0,
            format!(
                "unsupported format-version: dump is v{}, this binary expects v{}",
                header.format_version, FORMAT_VERSION
            ),
        ));
    }
    if !saw_end {
        return Err(err(0, "dump did not end with `# end` marker"));
    }

    Ok(DumpDocument { header, records })
}

fn parse_header_line(
    body: &str,
    header: &mut DumpHeader,
    saw_version: &mut bool,
    lineno: usize,
) -> Result<(), ParseError> {
    let (key, value) = body
        .split_once(':')
        .ok_or_else(|| err(lineno, format!("malformed header line: {body:?}")))?;
    let key = key.trim();
    let value = value.trim();
    match key {
        "format-version" => {
            header.format_version = value
                .parse::<u32>()
                .map_err(|e| err(lineno, format!("bad format-version: {e}")))?;
            *saw_version = true;
        }
        "generator-tool" => header.generator_tool = value.to_string(),
        "generator-opensta" => header.generator_opensta = value.to_string(),
        "input-files" => {
            header.input_files = value
                .split(',')
                .map(|s| s.trim().to_string())
                .filter(|s| !s.is_empty())
                .collect();
        }
        // Unrecognised header keys are tolerated — the format reserves the
        // right to add metadata. Strict consumers can re-read the header.
        _ => {}
    }
    Ok(())
}

fn parse_record(kind: &str, fields: &[&str], lineno: usize) -> Result<DumpRecord, ParseError> {
    match kind {
        "CORNER" => parse_corner(fields, lineno).map(DumpRecord::Corner),
        "ARC" => parse_arc(fields, lineno).map(DumpRecord::Arc),
        "INTERCONNECT" => parse_interconnect(fields, lineno).map(DumpRecord::Interconnect),
        "SETUP_HOLD" => parse_setup_hold(fields, lineno).map(DumpRecord::SetupHold),
        "CLOCK_ARRIVAL" => parse_clock_arrival(fields, lineno).map(DumpRecord::ClockArrival),
        "VENDOR_EXT" => parse_vendor_ext(fields, lineno).map(DumpRecord::VendorExt),
        other => Err(err(lineno, format!("unknown record kind {other:?}"))),
    }
}

fn parse_corner(fields: &[&str], lineno: usize) -> Result<CornerRecord, ParseError> {
    let [index, name, process, voltage, temperature] = field_array::<5>(fields, "CORNER", lineno)?;
    Ok(CornerRecord {
        index: parse_field(index, "index", lineno)?,
        name: name.to_string(),
        process: process.to_string(),
        voltage: parse_field(voltage, "voltage", lineno)?,
        temperature: parse_field(temperature, "temperature", lineno)?,
    })
}

fn parse_arc(fields: &[&str], lineno: usize) -> Result<ArcRecord, ParseError> {
    let [cell_instance, driver_pin, load_pin, corner_index, rise_min, rise_typ, rise_max, fall_min, fall_typ, fall_max, condition, origin] =
        field_array::<12>(fields, "ARC", lineno)?;
    Ok(ArcRecord {
        cell_instance: cell_instance.to_string(),
        driver_pin: driver_pin.to_string(),
        load_pin: load_pin.to_string(),
        corner_index: parse_field(corner_index, "corner_index", lineno)?,
        rise_min: parse_field(rise_min, "rise_min", lineno)?,
        rise_typ: parse_field(rise_typ, "rise_typ", lineno)?,
        rise_max: parse_field(rise_max, "rise_max", lineno)?,
        fall_min: parse_field(fall_min, "fall_min", lineno)?,
        fall_typ: parse_field(fall_typ, "fall_typ", lineno)?,
        fall_max: parse_field(fall_max, "fall_max", lineno)?,
        condition: condition.to_string(),
        origin: parse_field(origin, "origin", lineno)?,
    })
}

fn parse_interconnect(fields: &[&str], lineno: usize) -> Result<InterconnectRecord, ParseError> {
    let [net, from_pin, to_pin, corner_index, min, typ, max, origin] =
        field_array::<8>(fields, "INTERCONNECT", lineno)?;
    Ok(InterconnectRecord {
        net: net.to_string(),
        from_pin: from_pin.to_string(),
        to_pin: to_pin.to_string(),
        corner_index: parse_field(corner_index, "corner_index", lineno)?,
        min: parse_field(min, "min", lineno)?,
        typ: parse_field(typ, "typ", lineno)?,
        max: parse_field(max, "max", lineno)?,
        origin: parse_field(origin, "origin", lineno)?,
    })
}

fn parse_setup_hold(fields: &[&str], lineno: usize) -> Result<SetupHoldRecord, ParseError> {
    let [cell_instance, d_pin, clk_pin, edge, corner_index, setup_min, setup_typ, setup_max, hold_min, hold_typ, hold_max, condition, origin] =
        field_array::<13>(fields, "SETUP_HOLD", lineno)?;
    Ok(SetupHoldRecord {
        cell_instance: cell_instance.to_string(),
        d_pin: d_pin.to_string(),
        clk_pin: clk_pin.to_string(),
        edge: parse_field(edge, "edge", lineno)?,
        corner_index: parse_field(corner_index, "corner_index", lineno)?,
        setup_min: parse_field(setup_min, "setup_min", lineno)?,
        setup_typ: parse_field(setup_typ, "setup_typ", lineno)?,
        setup_max: parse_field(setup_max, "setup_max", lineno)?,
        hold_min: parse_field(hold_min, "hold_min", lineno)?,
        hold_typ: parse_field(hold_typ, "hold_typ", lineno)?,
        hold_max: parse_field(hold_max, "hold_max", lineno)?,
        condition: condition.to_string(),
        origin: parse_field(origin, "origin", lineno)?,
    })
}

fn parse_clock_arrival(fields: &[&str], lineno: usize) -> Result<ClockArrivalRecord, ParseError> {
    let [cell_instance, clk_pin, corner_index, min, typ, max, origin] =
        field_array::<7>(fields, "CLOCK_ARRIVAL", lineno)?;
    Ok(ClockArrivalRecord {
        cell_instance: cell_instance.to_string(),
        clk_pin: clk_pin.to_string(),
        corner_index: parse_field(corner_index, "corner_index", lineno)?,
        min: parse_field(min, "min", lineno)?,
        typ: parse_field(typ, "typ", lineno)?,
        max: parse_field(max, "max", lineno)?,
        origin: parse_field(origin, "origin", lineno)?,
    })
}

fn parse_vendor_ext(fields: &[&str], lineno: usize) -> Result<VendorExtRecord, ParseError> {
    let [source, source_tool, kind, payload] = field_array::<4>(fields, "VENDOR_EXT", lineno)?;
    Ok(VendorExtRecord {
        source: source.to_string(),
        source_tool: source_tool.to_string(),
        kind: kind.to_string(),
        raw_payload_b64: payload.to_string(),
    })
}

fn field_array<'a, const N: usize>(
    fields: &[&'a str],
    record: &str,
    lineno: usize,
) -> Result<[&'a str; N], ParseError> {
    if fields.len() != N {
        return Err(err(
            lineno,
            format!("{record}: expected {N} fields, got {}", fields.len()),
        ));
    }
    Ok(std::array::from_fn(|i| fields[i]))
}

fn parse_field<T>(s: &str, name: &str, lineno: usize) -> Result<T, ParseError>
where
    T: FromStr,
    T::Err: std::fmt::Display,
{
    s.parse::<T>()
        .map_err(|e| err(lineno, format!("{name} = {s:?}: {e}")))
}

fn err(line: usize, message: impl Into<String>) -> ParseError {
    ParseError {
        line,
        message: message.into(),
    }
}
