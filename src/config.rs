// SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Jacquard project configuration file (`jacquard.toml`) support.
//!
//! Provides optional TOML-based project configuration that stores design
//! parameters, mapping settings, and simulation options. CLI arguments
//! always override config file values.

use std::path::{Path, PathBuf};

use serde::Deserialize;

/// Jacquard project configuration loaded from `jacquard.toml`.
#[derive(Debug, Deserialize, Default)]
#[serde(default)]
pub struct JacquardConfig {
    pub design: DesignConfig,
    pub map: MapConfig,
    pub sim: SimConfig,
    pub cosim: CosimConfig,
}

/// Shared design parameters used across all subcommands.
#[derive(Debug, Deserialize, Default)]
#[serde(default)]
pub struct DesignConfig {
    /// Gate-level Verilog netlist path.
    pub netlist: Option<PathBuf>,
    /// Top module name (auto-detected if omitted).
    pub top_module: Option<String>,
    /// Liberty timing library path.
    pub liberty: Option<PathBuf>,
}

/// Partition mapping settings for `jacquard map`.
#[derive(Debug, Deserialize, Default)]
#[serde(default)]
pub struct MapConfig {
    /// Output partition file path.
    pub output: Option<PathBuf>,
    /// Level split thresholds for deep circuits.
    pub level_split: Vec<usize>,
    /// Maximum stage degradation layers.
    pub max_stage_degrad: Option<usize>,
    /// Enable X-propagation analysis.
    pub xprop: Option<bool>,
}

/// Simulation settings for `jacquard sim`.
#[derive(Debug, Deserialize, Default)]
#[serde(default)]
pub struct SimConfig {
    /// Partition file path. Defaults to `map.output` if not set.
    pub gemparts: Option<PathBuf>,
    /// Input VCD path.
    pub input_vcd: Option<PathBuf>,
    /// Output VCD path.
    pub output_vcd: Option<PathBuf>,
    /// Number of GPU blocks. Omit for auto-detect.
    pub num_blocks: Option<usize>,
    /// Input VCD scope path.
    pub input_vcd_scope: Option<String>,
    /// Output VCD scope path.
    pub output_vcd_scope: Option<String>,
    /// Maximum simulation cycles.
    pub max_cycles: Option<usize>,
    /// Verify GPU results against CPU baseline.
    pub check_with_cpu: Option<bool>,
    /// Enable X-propagation.
    pub xprop: Option<bool>,
    /// JSON file for display format strings (legacy).
    pub json_path: Option<PathBuf>,
    /// SDF timing back-annotation settings.
    pub sdf: Option<SdfConfig>,
    /// Post-simulation timing analysis settings.
    pub timing: Option<TimingConfig>,
}

/// SDF timing back-annotation configuration.
#[derive(Debug, Deserialize, Default)]
#[serde(default)]
pub struct SdfConfig {
    /// SDF file path.
    pub file: Option<PathBuf>,
    /// SDF corner: "min", "typ", or "max".
    pub corner: Option<String>,
    /// Enable SDF debug output.
    pub debug: Option<bool>,
}

/// Post-simulation timing analysis configuration.
#[derive(Debug, Deserialize, Default)]
#[serde(default)]
pub struct TimingConfig {
    /// Enable timing analysis.
    pub enabled: Option<bool>,
    /// Clock period in picoseconds.
    pub clock_period: Option<u64>,
    /// Report all violations.
    pub report_violations: Option<bool>,
}

/// Co-simulation settings for `jacquard cosim`.
#[derive(Debug, Deserialize, Default)]
#[serde(default)]
pub struct CosimConfig {
    /// Testbench JSON config path.
    pub config: Option<PathBuf>,
    /// Number of GPU blocks.
    pub num_blocks: Option<usize>,
    /// Maximum simulation cycles.
    pub max_cycles: Option<usize>,
    /// Clock period in picoseconds.
    pub clock_period: Option<u64>,
}

impl JacquardConfig {
    /// Discover a `jacquard.toml` config file by searching CWD and parent directories.
    ///
    /// Returns the parsed config and the path to the config file, or `None` if not found.
    pub fn discover() -> Option<(Self, PathBuf)> {
        let cwd = std::env::current_dir().ok()?;
        let mut dir = cwd.as_path();
        loop {
            let candidate = dir.join("jacquard.toml");
            if candidate.exists() {
                match Self::load(&candidate) {
                    Ok(mut config) => {
                        let config_dir = candidate.parent().unwrap_or(Path::new("."));
                        config.resolve_paths(config_dir);
                        return Some((config, candidate));
                    }
                    Err(e) => {
                        clilog::warn!("Found jacquard.toml but failed to parse: {}", e);
                        return None;
                    }
                }
            }
            dir = dir.parent()?;
        }
    }

    /// Load configuration from a specific path.
    pub fn load(path: &Path) -> Result<Self, String> {
        let content = std::fs::read_to_string(path)
            .map_err(|e| format!("Failed to read {}: {}", path.display(), e))?;
        toml::from_str(&content)
            .map_err(|e| format!("Failed to parse {}: {}", path.display(), e))
    }

    /// Resolve relative paths against the config file's directory.
    pub fn resolve_paths(&mut self, config_dir: &Path) {
        resolve_opt_path(&mut self.design.netlist, config_dir);
        resolve_opt_path(&mut self.design.liberty, config_dir);
        resolve_opt_path(&mut self.map.output, config_dir);
        resolve_opt_path(&mut self.sim.gemparts, config_dir);
        resolve_opt_path(&mut self.sim.input_vcd, config_dir);
        resolve_opt_path(&mut self.sim.output_vcd, config_dir);
        resolve_opt_path(&mut self.sim.json_path, config_dir);
        if let Some(ref mut sdf) = self.sim.sdf {
            resolve_opt_path(&mut sdf.file, config_dir);
        }
        resolve_opt_path(&mut self.cosim.config, config_dir);
    }

    /// Get the effective gemparts path, falling back to map.output.
    pub fn effective_gemparts(&self) -> Option<&PathBuf> {
        self.sim.gemparts.as_ref().or(self.map.output.as_ref())
    }
}

/// Resolve a relative path against a base directory. Absolute paths are unchanged.
fn resolve_opt_path(path: &mut Option<PathBuf>, base: &Path) {
    if let Some(ref mut p) = path {
        if p.is_relative() {
            *p = base.join(&*p);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_config() {
        let config: JacquardConfig = toml::from_str("").unwrap();
        assert!(config.design.netlist.is_none());
        assert!(config.map.level_split.is_empty());
        assert!(config.sim.num_blocks.is_none());
    }

    #[test]
    fn test_minimal_config() {
        let toml_str = r#"
[design]
netlist = "gatelevel.gv"

[map]
output = "result.gemparts"

[sim]
input_vcd = "input.vcd"
output_vcd = "output.vcd"
"#;
        let config: JacquardConfig = toml::from_str(toml_str).unwrap();
        assert_eq!(
            config.design.netlist.as_ref().unwrap(),
            &PathBuf::from("gatelevel.gv")
        );
        assert_eq!(
            config.map.output.as_ref().unwrap(),
            &PathBuf::from("result.gemparts")
        );
    }

    #[test]
    fn test_full_config() {
        let toml_str = r#"
[design]
netlist = "build/gatelevel.gv"
top_module = "my_top"
liberty = "/opt/pdk/lib.lib"

[map]
output = "build/result.gemparts"
level_split = [20, 40]
max_stage_degrad = 1
xprop = true

[sim]
input_vcd = "test/input.vcd"
output_vcd = "test/output.vcd"
num_blocks = 128
max_cycles = 1000
check_with_cpu = true
xprop = true

[sim.sdf]
file = "build/design.sdf"
corner = "typ"
debug = false

[sim.timing]
enabled = true
clock_period = 1000
report_violations = true

[cosim]
config = "sim_config.json"
num_blocks = 64
max_cycles = 500000
clock_period = 40000
"#;
        let config: JacquardConfig = toml::from_str(toml_str).unwrap();
        assert_eq!(config.map.level_split, vec![20, 40]);
        assert_eq!(config.sim.num_blocks, Some(128));
        assert!(config.sim.sdf.is_some());
        assert_eq!(config.cosim.clock_period, Some(40000));
    }

    #[test]
    fn test_path_resolution() {
        let toml_str = r#"
[design]
netlist = "build/gatelevel.gv"
liberty = "/absolute/path/lib.lib"

[map]
output = "build/result.gemparts"
"#;
        let mut config: JacquardConfig = toml::from_str(toml_str).unwrap();
        config.resolve_paths(Path::new("/project/dir"));

        assert_eq!(
            config.design.netlist.as_ref().unwrap(),
            &PathBuf::from("/project/dir/build/gatelevel.gv")
        );
        // Absolute paths should be unchanged
        assert_eq!(
            config.design.liberty.as_ref().unwrap(),
            &PathBuf::from("/absolute/path/lib.lib")
        );
        assert_eq!(
            config.map.output.as_ref().unwrap(),
            &PathBuf::from("/project/dir/build/result.gemparts")
        );
    }

    #[test]
    fn test_effective_gemparts_fallback() {
        let toml_str = r#"
[map]
output = "result.gemparts"
"#;
        let config: JacquardConfig = toml::from_str(toml_str).unwrap();
        assert_eq!(
            config.effective_gemparts().unwrap(),
            &PathBuf::from("result.gemparts")
        );
    }
}
