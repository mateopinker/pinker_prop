# Pinker Prop

Preliminary hover propeller sizing for small multirotors using actuator disk theory and blade element theory.

## Overview

This project estimates inflow velocity, spanwise thrust loading, relative flow conditions, chord distribution, angle of attack, Reynolds number, and geometric pitch for a hover propeller concept.

The physics in [pinker_prop.py](pinker_prop.py) is human-built. The interface in [pinker_prop_ui.py](pinker_prop_ui.py) is kept separate so UI changes do not modify the aerodynamic model.

## Method

The current implementation combines:

- momentum theory for induced hover inflow
- blade element theory for sectional loading
- airfoil polar interpolation for local aerodynamic coefficients
- a bracketed chord search to match elemental thrust

## Files

[pinker_prop.py](pinker_prop.py) contains the solver.

[pinker_prop_ui.py](pinker_prop_ui.py) provides a separate desktop UI for the same calculation flow.

## Run

```bash
python3 pinker_prop.py
python3 pinker_prop_ui.py
```

## Scope

This is a first-pass design tool, not a final propeller design environment. Present assumptions include:

- hover-only analysis
- incompressible flow
- one airfoil across the full span
- sectional 2D polar behavior
- no induced-angle correction
- no structural or aeroelastic model
