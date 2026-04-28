# dump_timing.tcl — driver for opensta-to-ir.
#
# Reads inputs, links the design, emits a dump in the format consumed by
# crates/opensta-to-ir/src/dump.rs.
#
# Phase 0 WS2 (Phase 2.1-followup): subprocess plumbing only. The Tcl
# script loads inputs and emits a valid header + a single CORNER record +
# the `# end` marker, but does *not* yet enumerate timing arcs — that is
# the next slice. The Rust side already supports ARC / INTERCONNECT /
# SETUP_HOLD parsing, so extending this driver is additive.
#
# Inputs are passed via environment variables to keep the Tcl simple and
# avoid quoting issues:
#   JACQUARD_DUMP_PATH        Output dump file path (required).
#   JACQUARD_LIBERTY_FILES    Newline-separated paths (required).
#   JACQUARD_VERILOG_FILES    Newline-separated paths (required).
#   JACQUARD_SDF_FILE         Optional.
#   JACQUARD_SPEF_FILE        Optional.
#   JACQUARD_SDC_FILE         Optional.
#   JACQUARD_TOP              Top-level module name (required).
#   JACQUARD_GENERATOR_VERSION  Crate version string for the dump header.

proc require_env { name } {
    global env
    if { ![info exists env($name)] || [string length $env($name)] == 0 } {
        puts stderr "dump_timing.tcl: required env var $name is unset"
        exit 1
    }
    return $env($name)
}

proc optional_env { name } {
    global env
    if { [info exists env($name)] } {
        return $env($name)
    }
    return ""
}

set dump_path [require_env JACQUARD_DUMP_PATH]
set liberty_files [split [require_env JACQUARD_LIBERTY_FILES] "\n"]
set verilog_files [split [require_env JACQUARD_VERILOG_FILES] "\n"]
set sdf_file [optional_env JACQUARD_SDF_FILE]
set spef_file [optional_env JACQUARD_SPEF_FILE]
set sdc_file [optional_env JACQUARD_SDC_FILE]
set top [require_env JACQUARD_TOP]
set generator_version [optional_env JACQUARD_GENERATOR_VERSION]
if { [string length $generator_version] == 0 } {
    set generator_version "unknown"
}

foreach lib $liberty_files {
    if { [string length $lib] > 0 } {
        read_liberty $lib
    }
}
foreach v $verilog_files {
    if { [string length $v] > 0 } {
        read_verilog $v
    }
}
link_design $top

if { [string length $sdf_file] > 0 } {
    read_sdf $sdf_file
}
if { [string length $spef_file] > 0 } {
    read_spef $spef_file
}
if { [string length $sdc_file] > 0 } {
    read_sdc $sdc_file
}

# Build the input-files header — comma-separated, used for IR provenance.
set input_files [list]
foreach lib $liberty_files {
    if { [string length $lib] > 0 } {
        lappend input_files $lib
    }
}
foreach v $verilog_files {
    if { [string length $v] > 0 } {
        lappend input_files $v
    }
}
if { [string length $sdf_file] > 0 } { lappend input_files $sdf_file }
if { [string length $spef_file] > 0 } { lappend input_files $spef_file }
if { [string length $sdc_file] > 0 } { lappend input_files $sdc_file }

set fh [open $dump_path "w"]
puts $fh "# format-version: 1"
puts $fh "# generator-tool: opensta-to-ir $generator_version"
puts $fh "# generator-opensta: OpenSTA"
puts $fh "# input-files: [join $input_files {, }]"

# Phase 2.1: emit a single default corner. Multi-corner is Phase 2.4.
puts $fh "CORNER\t0\tdefault\ttt\t1.0\t25.0"

# Phase 2.1-followup-B will iterate the timing graph and emit ARC /
# INTERCONNECT / SETUP_HOLD records here. For now, the dump is
# deliberately empty of those — the Rust side will treat that as
# "no arcs", which the binary's --min-arcs / --allow-empty-parse logic
# handles correctly for tests.

puts $fh "# end"
close $fh
exit 0
