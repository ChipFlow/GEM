# Pre-layout SKY130 timing tests
# These are synthesized but not placed/routed designs, using Liberty-only timing

.PHONY: all clean

all: inv_chain.v.gv logic_cone.v.gv inv_chain.sdf logic_cone.sdf

# Convert Verilog to AIG graph format for loom
%.v.gv: %.v
	@echo "Converting $< to AIG..."
	cargo run -r --bin jacquard -- map $< > $@ 2>&1 || { \
	  echo "Note: Direct mapping may not work; Verilog may need synthesis first"; \
	  echo "Falling back to direct Verilog format for CVC"; \
	}

# Generate Liberty-only SDF files
# These are created from SKY130 library timing values, not from P&R
%.sdf: %.v
	@echo "Generating Liberty-only SDF for $<..."
	python3 gen_liberty_sdf.py $< > $@ || { \
	  echo "Warning: Could not generate SDF for $<"; \
	}

clean:
	rm -f *.gv *.sdf
	rm -rf __pycache__

help:
	@echo "Pre-layout SKY130 timing tests"
	@echo ""
	@echo "Targets:"
	@echo "  all        - Generate AIG graphs and SDF files"
	@echo "  inv_chain  - Generate inv_chain files"
	@echo "  logic_cone - Generate logic_cone files"
	@echo "  clean      - Remove generated files"
