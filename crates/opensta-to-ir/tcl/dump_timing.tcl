# dump_timing.tcl — driver for opensta-to-ir.
#
# Reads inputs, links the design, walks OpenSTA's timing graph, and
# emits a dump in the format consumed by crates/opensta-to-ir/src/dump.rs.
#
# Phase 0 WS2 (Phase 2.1-followup-B): single-corner CORNER + ARC records
# only. INTERCONNECT, SETUP_HOLD, multi-corner, and vendor extensions
# are later slices. The Rust side already supports the full record set.
#
# Inputs are passed via environment variables to keep the Tcl simple and
# avoid quoting issues. See src/opensta.rs for the producer side.

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
    if { [string length $lib] > 0 } { read_liberty $lib }
}
foreach v $verilog_files {
    if { [string length $v] > 0 } { read_verilog $v }
}
link_design $top

if { [string length $sdf_file] > 0 } { read_sdf $sdf_file }
if { [string length $spef_file] > 0 } { read_spef $spef_file }
if { [string length $sdc_file] > 0 } { read_sdc $sdc_file }

# OpenSTA's `arc_delays` returns delays in seconds (its internal unit)
# regardless of the design's time_unit. Scale to picoseconds for the IR.
set SECONDS_TO_PS 1e12

# Origin label — SDF-asserted if back-annotation was loaded, otherwise
# Liberty-computed. Placed in every emitted ARC's origin field.
set origin "Computed"
if { [string length $sdf_file] > 0 } { set origin "Asserted" }

# Build the input-files header — comma-separated, used for IR provenance.
set input_files [list]
foreach lib $liberty_files {
    if { [string length $lib] > 0 } { lappend input_files $lib }
}
foreach v $verilog_files {
    if { [string length $v] > 0 } { lappend input_files $v }
}
if { [string length $sdf_file] > 0 } { lappend input_files $sdf_file }
if { [string length $spef_file] > 0 } { lappend input_files $spef_file }
if { [string length $sdc_file] > 0 } { lappend input_files $sdc_file }

# ---------------------------------------------------------------------------
# Walk the timing graph and aggregate arcs by (cell, from_pin, to_pin).
#
# Each Edge holds a TimingArcSet of multiple TimingArc objects, one per
# (input_transition, output_transition) pair. We bucket by the output
# transition (rise/fall) and take the max delay across input transitions
# — matches Jacquard's existing pessimism convention. Per-input-transition
# detail is a richer feature for a future phase.
# ---------------------------------------------------------------------------

# arcs is a Tcl dict: key "cell|from|to" → dict with cell_inst from_pin
# to_pin rise_min rise_max fall_min fall_max
set arcs [dict create]

# Take the larger of two values when one may be empty.
proc max_or_set { current candidate } {
    if { $current eq "" } { return $candidate }
    if { $candidate > $current } { return $candidate }
    return $current
}

set vit [::sta::vertex_iterator]
while { [$vit has_next] } {
    set vertex [$vit next]
    set eit [$vertex in_edge_iterator]
    while { [$eit has_next] } {
        set edge [$eit next]
        set role [$edge role]
        # Skip setup / hold / recovery / removal / width — those are
        # SETUP_HOLD records, populated in a future slice.
        if { [::sta::timing_role_is_check $role] } continue

        set from_pin [$edge from_pin]
        set to_pin [$edge to_pin]
        set from_full [get_full_name $from_pin]
        set to_full [get_full_name $to_pin]

        # We only emit cell-internal arcs in this slice. Both ends must
        # be on the same instance.
        set last_to [string last "/" $to_full]
        set last_from [string last "/" $from_full]
        if { $last_to < 0 || $last_from < 0 } continue
        set cell_inst [string range $to_full 0 [expr {$last_to - 1}]]
        set from_inst [string range $from_full 0 [expr {$last_from - 1}]]
        if { $cell_inst ne $from_inst } continue
        set to_name [string range $to_full [expr {$last_to + 1}] end]
        set from_name [string range $from_full [expr {$last_from + 1}] end]

        set key "$cell_inst|$from_name|$to_name"
        if { ![dict exists $arcs $key] } {
            dict set arcs $key cell_inst $cell_inst
            dict set arcs $key from_pin $from_name
            dict set arcs $key to_pin $to_name
            dict set arcs $key rise_min ""
            dict set arcs $key rise_max ""
            dict set arcs $key fall_min ""
            dict set arcs $key fall_max ""
        }

        foreach arc [$edge timing_arcs] {
            set delays [$edge arc_delays $arc]
            # delays = [min max] for the default single scene, in seconds.
            set d_min [expr {[lindex $delays 0] * $SECONDS_TO_PS}]
            set d_max [expr {[lindex $delays 1] * $SECONDS_TO_PS}]
            set to_edge [$arc to_edge_name]
            if { $to_edge eq "rise" } {
                dict set arcs $key rise_min [max_or_set [dict get $arcs $key rise_min] $d_min]
                dict set arcs $key rise_max [max_or_set [dict get $arcs $key rise_max] $d_max]
            } elseif { $to_edge eq "fall" } {
                dict set arcs $key fall_min [max_or_set [dict get $arcs $key fall_min] $d_min]
                dict set arcs $key fall_max [max_or_set [dict get $arcs $key fall_max] $d_max]
            }
        }
    }
    $eit finish
}
$vit finish

# ---------------------------------------------------------------------------
# Emit the dump file.
# ---------------------------------------------------------------------------

set fh [open $dump_path "w"]
puts $fh "# format-version: 1"
puts $fh "# generator-tool: opensta-to-ir $generator_version"
puts $fh "# generator-opensta: OpenSTA"
puts $fh "# input-files: [join $input_files {, }]"

# Single default corner for now. Multi-corner is Phase 2.4.
puts $fh "CORNER\t0\tdefault\ttt\t1.0\t25.0"

dict for {key data} $arcs {
    set cell_inst [dict get $data cell_inst]
    set from_pin [dict get $data from_pin]
    set to_pin [dict get $data to_pin]
    set rise_min [dict get $data rise_min]
    set rise_max [dict get $data rise_max]
    set fall_min [dict get $data fall_min]
    set fall_max [dict get $data fall_max]

    # Default missing rise or fall to 0 — represents an arc that only
    # produces one output transition.
    if { $rise_min eq "" } { set rise_min 0.0; set rise_max 0.0 }
    if { $fall_min eq "" } { set fall_min 0.0; set fall_max 0.0 }
    set rise_typ [expr {($rise_min + $rise_max) / 2.0}]
    set fall_typ [expr {($fall_min + $fall_max) / 2.0}]

    puts $fh [format "ARC\t%s\t%s\t%s\t0\t%g\t%g\t%g\t%g\t%g\t%g\t\t%s" \
        $cell_inst $from_pin $to_pin \
        $rise_min $rise_typ $rise_max \
        $fall_min $fall_typ $fall_max \
        $origin]
}

puts $fh "# end"
close $fh
exit 0
