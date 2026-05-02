# dump_timing.tcl — driver for opensta-to-ir.
#
# Reads inputs, links the design, walks OpenSTA's timing graph, and
# emits a dump in the format consumed by crates/opensta-to-ir/src/dump.rs.
#
# WS2 (2.2 + 2.3 + 2.4) + Pillar B Stage 1: emits CORNER + ARC +
# SETUP_HOLD + INTERCONNECT + CLOCK_ARRIVAL records, one per corner per
# (cell, …) key. The Rust builder dedupes per-corner emissions into
# single IR records carrying multi-corner [TimingValue] vectors.
#
# Inputs are passed via environment variables to keep the Tcl simple and
# avoid quoting issues. See src/opensta.rs for the producer side.
#
# JACQUARD_CORNERS: TAB-separated rows, NEWLINE-separated. Each row:
#   name TAB process TAB voltage TAB temperature TAB lib_path1 [TAB lib_path2 ...]
# At least one row is required.

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
set verilog_files [split [require_env JACQUARD_VERILOG_FILES] "\n"]
set sdf_file [optional_env JACQUARD_SDF_FILE]
set spef_file [optional_env JACQUARD_SPEF_FILE]
set sdc_file [optional_env JACQUARD_SDC_FILE]
set top [require_env JACQUARD_TOP]
set generator_version [optional_env JACQUARD_GENERATOR_VERSION]
if { [string length $generator_version] == 0 } {
    set generator_version "unknown"
}

# Parse the JACQUARD_CORNERS env var — see header for format.
set corners_raw [require_env JACQUARD_CORNERS]
set corner_rows [split $corners_raw "\n"]
set corner_names [list]
set corner_meta [dict create] ;# name -> {process voltage temperature [lib...]}
foreach row $corner_rows {
    if { [string length $row] == 0 } continue
    set fields [split $row "\t"]
    if { [llength $fields] < 5 } {
        puts stderr "JACQUARD_CORNERS row malformed: $row"
        exit 1
    }
    set cname [lindex $fields 0]
    set cprocess [lindex $fields 1]
    set cvoltage [lindex $fields 2]
    set ctemp [lindex $fields 3]
    set clibs [lrange $fields 4 end]
    lappend corner_names $cname
    dict set corner_meta $cname process $cprocess
    dict set corner_meta $cname voltage $cvoltage
    dict set corner_meta $cname temperature $ctemp
    dict set corner_meta $cname libs $clibs
}
if { [llength $corner_names] == 0 } {
    puts stderr "JACQUARD_CORNERS: at least one corner is required"
    exit 1
}

# Define corners up front, then read each corner's Liberty files into it.
# `define_corners` takes the names as positional args; we expand the list.
eval [list define_corners] $corner_names
foreach cname $corner_names {
    foreach lib [dict get $corner_meta $cname libs] {
        if { [string length $lib] > 0 } {
            read_liberty -corner $cname $lib
        }
    }
}
foreach v $verilog_files {
    if { [string length $v] > 0 } { read_verilog $v }
}
link_design $top

# corner_index map — position in the list = the corner_index emitted to
# the dump (matches the IR builder's per-record [TimingValue] order).
set corner_index_of [dict create]
set ci 0
foreach cname $corner_names {
    dict set corner_index_of $cname $ci
    incr ci
}

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
foreach cname $corner_names {
    foreach lib [dict get $corner_meta $cname libs] {
        if { [string length $lib] > 0 } { lappend input_files $lib }
    }
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

# arcs:      cell-internal delay arcs, keyed "cell|from|to|corner_idx".
# checks:    cell-internal setup/hold checks, keyed "cell|d|clk|edge|corner_idx".
# wires:     net interconnect delays, keyed "net|from_pin|to_pin|corner_idx".
# arrivals:  per-DFF clock arrivals, keyed "cell|clk_pin|corner_idx".
#
# `[edge arc_delays $arc]` returns a flat list of length `2 * num_scenes`,
# layout `c0_min, c0_max, c1_min, c1_max, ...` (matches OpenSTA's
# `Sta::scenes()` order). Per-corner records are emitted by indexing
# the flat list at `[ci*2]` and `[ci*2+1]`. Per-corner CLOCK_ARRIVAL
# uses `set_cmd_scene` + `vertex_worst_arrival_path`.
set arcs [dict create]
set checks [dict create]
set wires [dict create]
set arrivals [dict create]
set num_corners [llength $corner_names]

# Take the larger of two values when one may be empty.
proc max_or_set { current candidate } {
    if { $current eq "" } { return $candidate }
    if { $candidate > $current } { return $candidate }
    return $current
}

# Pin-pair extractor — returns {cell_inst from_name to_name} for a
# (from_pin, to_pin) edge whose endpoints are on the same instance, or
# the empty list if either pin is on a top-level port or the pins are
# on different instances.
proc pin_pair { from_pin to_pin } {
    set from_full [get_full_name $from_pin]
    set to_full [get_full_name $to_pin]
    set last_to [string last "/" $to_full]
    set last_from [string last "/" $from_full]
    if { $last_to < 0 || $last_from < 0 } { return {} }
    set cell_inst [string range $to_full 0 [expr {$last_to - 1}]]
    set from_inst [string range $from_full 0 [expr {$last_from - 1}]]
    if { $cell_inst ne $from_inst } { return {} }
    set to_name [string range $to_full [expr {$last_to + 1}] end]
    set from_name [string range $from_full [expr {$last_from + 1}] end]
    return [list $cell_inst $from_name $to_name]
}

set vit [::sta::vertex_iterator]
while { [$vit has_next] } {
    set vertex [$vit next]
    set eit [$vertex in_edge_iterator]
    while { [$eit has_next] } {
        set edge [$eit next]
        set role [$edge role]

        # Wire delay edge — connects a driver pin on one instance to a
        # load pin on another. Emit an INTERCONNECT record per
        # (net, from_pin, to_pin); take the max delay across the edge's
        # timing arcs to match the existing rise/fall pessimism.
        if { $role eq "wire" } {
            set ef_pin [$edge from_pin]
            set et_pin [$edge to_pin]
            set net [$ef_pin net]
            if { $net eq "NULL" || $net eq "" } continue
            set net_name [get_full_name $net]
            set from_full [get_full_name $ef_pin]
            set to_full [get_full_name $et_pin]
            for { set ci 0 } { $ci < $num_corners } { incr ci } {
                set d_min ""
                set d_max ""
                foreach arc [$edge timing_arcs] {
                    set delays [$edge arc_delays $arc]
                    set arc_min [expr {[lindex $delays [expr {$ci * 2}]] * $SECONDS_TO_PS}]
                    set arc_max [expr {[lindex $delays [expr {$ci * 2 + 1}]] * $SECONDS_TO_PS}]
                    set d_min [max_or_set $d_min $arc_min]
                    set d_max [max_or_set $d_max $arc_max]
                }
                if { $d_min eq "" } continue
                set wkey "$net_name|$from_full|$to_full|$ci"
                if { ![dict exists $wires $wkey] } {
                    dict set wires $wkey net $net_name
                    dict set wires $wkey from_pin $from_full
                    dict set wires $wkey to_pin $to_full
                    dict set wires $wkey corner_idx $ci
                    dict set wires $wkey min $d_min
                    dict set wires $wkey max $d_max
                } else {
                    dict set wires $wkey min [max_or_set [dict get $wires $wkey min] $d_min]
                    dict set wires $wkey max [max_or_set [dict get $wires $wkey max] $d_max]
                }
            }
            continue
        }

        set pair [pin_pair [$edge from_pin] [$edge to_pin]]
        if { [llength $pair] == 0 } continue
        lassign $pair cell_inst from_name to_name

        if { $role eq "setup" || $role eq "hold" } {
            # OpenSTA's setup/hold timing-graph edges point CLK → D, so
            # in-edges of D's vertex have from_name=CLK and to_name=D.
            # Map back to the SDF convention (d_pin, clk_pin) for the IR.
            set d_pin_name $to_name
            set clk_pin_name $from_name
            foreach arc [$edge timing_arcs] {
                set delays [$edge arc_delays $arc]
                set to_edge [$arc to_edge_name]
                set edge_label "Posedge"
                if { $to_edge eq "fall" } { set edge_label "Negedge" }
                for { set ci 0 } { $ci < $num_corners } { incr ci } {
                    set d_min [expr {[lindex $delays [expr {$ci * 2}]] * $SECONDS_TO_PS}]
                    set d_max [expr {[lindex $delays [expr {$ci * 2 + 1}]] * $SECONDS_TO_PS}]
                    set key "$cell_inst|$d_pin_name|$clk_pin_name|$edge_label|$ci"
                    if { ![dict exists $checks $key] } {
                        dict set checks $key cell_inst $cell_inst
                        dict set checks $key d_pin $d_pin_name
                        dict set checks $key clk_pin $clk_pin_name
                        dict set checks $key edge $edge_label
                        dict set checks $key corner_idx $ci
                        dict set checks $key setup_min ""
                        dict set checks $key setup_max ""
                        dict set checks $key hold_min ""
                        dict set checks $key hold_max ""
                    }
                    if { $role eq "setup" } {
                        dict set checks $key setup_min [max_or_set [dict get $checks $key setup_min] $d_min]
                        dict set checks $key setup_max [max_or_set [dict get $checks $key setup_max] $d_max]
                    } else {
                        dict set checks $key hold_min [max_or_set [dict get $checks $key hold_min] $d_min]
                        dict set checks $key hold_max [max_or_set [dict get $checks $key hold_max] $d_max]
                    }
                }
            }
            continue
        }

        # Skip remaining timing checks (recovery / removal / width) —
        # those land in a later slice if needed.
        if { [::sta::timing_role_is_check $role] } continue

        # Delay arc — emit one entry per (cell, from, to, corner_idx).
        for { set ci 0 } { $ci < $num_corners } { incr ci } {
            set key "$cell_inst|$from_name|$to_name|$ci"
            if { ![dict exists $arcs $key] } {
                dict set arcs $key cell_inst $cell_inst
                dict set arcs $key from_pin $from_name
                dict set arcs $key to_pin $to_name
                dict set arcs $key corner_idx $ci
                dict set arcs $key rise_min ""
                dict set arcs $key rise_max ""
                dict set arcs $key fall_min ""
                dict set arcs $key fall_max ""
            }
            foreach arc [$edge timing_arcs] {
                set delays [$edge arc_delays $arc]
                set d_min [expr {[lindex $delays [expr {$ci * 2}]] * $SECONDS_TO_PS}]
                set d_max [expr {[lindex $delays [expr {$ci * 2 + 1}]] * $SECONDS_TO_PS}]
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
    }
    $eit finish
}
$vit finish

# ---------------------------------------------------------------------------
# Per-DFF clock arrival (Pillar B Stage 1).
#
# For each register's clock pin, walk the worst-case clock path and emit
# its arrival time in picoseconds. Origin is always `Computed` because
# arrival is always derived by the STA tool from the timing graph — SDF
# back-annotation feeds per-arc delays, not per-pin arrival.
#
# Per-pair CRPR is not captured here (it is a launch/capture credit, not a
# per-DFF property). Consumers treat the launch-side arrival as the
# reference (== 0). See `crates/timing-ir/schemas/timing_ir.fbs` and
# `docs/timing-model-extensions.md` Pillar B for the contract.
# ---------------------------------------------------------------------------

# Force the timing graph to be up to date before querying arrivals.
# `find_timing_cmd` is OpenSTA's SWIG-exposed timing-update primitive.
::sta::find_timing_cmd 1

foreach clk_pin [all_registers -clock_pins] {
    set pin_vertices [$clk_pin vertices]
    set vertex [lindex $pin_vertices 0]
    if { $vertex eq "" || $vertex eq "NULL" } continue

    set pin_full [get_full_name $clk_pin]
    set last_slash [string last "/" $pin_full]
    if { $last_slash < 0 } continue
    set cell_inst [string range $pin_full 0 [expr {$last_slash - 1}]]
    set local_pin [string range $pin_full [expr {$last_slash + 1}] end]

    set ci 0
    foreach cname $corner_names {
        ::sta::set_cmd_scene $cname
        set arr_min ""
        set arr_max ""
        if { ![catch { set path_min [::sta::vertex_worst_arrival_path $vertex "min"] } ] \
                && $path_min ne "" && $path_min ne "NULL" } {
            set arr_min [expr {[$path_min arrival] * $SECONDS_TO_PS}]
        }
        if { ![catch { set path_max [::sta::vertex_worst_arrival_path $vertex "max"] } ] \
                && $path_max ne "" && $path_max ne "NULL" } {
            set arr_max [expr {[$path_max arrival] * $SECONDS_TO_PS}]
        }
        if { $arr_min eq "" && $arr_max eq "" } { incr ci; continue }
        if { $arr_min eq "" } { set arr_min $arr_max }
        if { $arr_max eq "" } { set arr_max $arr_min }

        set akey "$cell_inst|$local_pin|$ci"
        dict set arrivals $akey cell_inst $cell_inst
        dict set arrivals $akey clk_pin $local_pin
        dict set arrivals $akey corner_idx $ci
        dict set arrivals $akey min $arr_min
        dict set arrivals $akey max $arr_max
        incr ci
    }
}

# ---------------------------------------------------------------------------
# Emit the dump file.
# ---------------------------------------------------------------------------

set fh [open $dump_path "w"]
puts $fh "# format-version: 1"
puts $fh "# generator-tool: opensta-to-ir $generator_version"
puts $fh "# generator-opensta: OpenSTA"
puts $fh "# input-files: [join $input_files {, }]"

# Emit one CORNER record per declared corner. Order matches the
# corner_index used in the per-record entries below.
set ci 0
foreach cname $corner_names {
    set cprocess [dict get $corner_meta $cname process]
    set cvoltage [dict get $corner_meta $cname voltage]
    set ctemp [dict get $corner_meta $cname temperature]
    puts $fh [format "CORNER\t%d\t%s\t%s\t%s\t%s" $ci $cname $cprocess $cvoltage $ctemp]
    incr ci
}

dict for {key data} $arcs {
    set cell_inst [dict get $data cell_inst]
    set from_pin [dict get $data from_pin]
    set to_pin [dict get $data to_pin]
    set corner_idx [dict get $data corner_idx]
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

    puts $fh [format "ARC\t%s\t%s\t%s\t%d\t%g\t%g\t%g\t%g\t%g\t%g\t\t%s" \
        $cell_inst $from_pin $to_pin $corner_idx \
        $rise_min $rise_typ $rise_max \
        $fall_min $fall_typ $fall_max \
        $origin]
}

dict for {key data} $checks {
    set cell_inst [dict get $data cell_inst]
    set d_pin [dict get $data d_pin]
    set clk_pin [dict get $data clk_pin]
    set edge_label [dict get $data edge]
    set corner_idx [dict get $data corner_idx]
    set setup_min [dict get $data setup_min]
    set setup_max [dict get $data setup_max]
    set hold_min [dict get $data hold_min]
    set hold_max [dict get $data hold_max]

    # Setup-only or hold-only entries default the missing side to 0.
    if { $setup_min eq "" } { set setup_min 0.0; set setup_max 0.0 }
    if { $hold_min eq "" } { set hold_min 0.0; set hold_max 0.0 }
    set setup_typ [expr {($setup_min + $setup_max) / 2.0}]
    set hold_typ [expr {($hold_min + $hold_max) / 2.0}]

    puts $fh [format "SETUP_HOLD\t%s\t%s\t%s\t%s\t%d\t%g\t%g\t%g\t%g\t%g\t%g\t\t%s" \
        $cell_inst $d_pin $clk_pin $edge_label $corner_idx \
        $setup_min $setup_typ $setup_max \
        $hold_min $hold_typ $hold_max \
        $origin]
}

dict for {key data} $wires {
    set net_name [dict get $data net]
    set from_pin [dict get $data from_pin]
    set to_pin [dict get $data to_pin]
    set corner_idx [dict get $data corner_idx]
    set d_min [dict get $data min]
    set d_max [dict get $data max]
    set d_typ [expr {($d_min + $d_max) / 2.0}]

    puts $fh [format "INTERCONNECT\t%s\t%s\t%s\t%d\t%g\t%g\t%g\t%s" \
        $net_name $from_pin $to_pin $corner_idx \
        $d_min $d_typ $d_max \
        $origin]
}

dict for {key data} $arrivals {
    set cell_inst [dict get $data cell_inst]
    set clk_pin [dict get $data clk_pin]
    set corner_idx [dict get $data corner_idx]
    set d_min [dict get $data min]
    set d_max [dict get $data max]
    set d_typ [expr {($d_min + $d_max) / 2.0}]

    # Origin is always Computed for clock arrivals — see header comment.
    puts $fh [format "CLOCK_ARRIVAL\t%s\t%s\t%d\t%g\t%g\t%g\tComputed" \
        $cell_inst $clk_pin $corner_idx \
        $d_min $d_typ $d_max]
}

puts $fh "# end"
close $fh
exit 0
