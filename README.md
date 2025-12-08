# SNU Spice

SNU Spice is an electronic circuit simulator that uses Modified Nodal Analysis to analyze circuits containing resistors and independent or dependent voltage and current sources, with support for transient simulations and simple nonlinear capacitor/inductor models. The project was developed at Shiv Nadar University under the supervision of Prof. Venkatnarayan Hariharan as part of the Opportunities for Undergraduate Research Program (OUR20240023) and as the ECE498 Minor Project for the Electrical & Computer Engineering degree. The implementation follows approaches described in Farid N. Najm’s book "Circuit Simulation".

## Table of Contents

- [Working](#working)
- [Installation and Running SNU Spice](#installation-and-running-snu-spice)
  - [Requirements](#requirements)
  - [Installation (CMake)](#installation-cmake)
  - [Running](#running)
- [Accepted Syntax of Circuit Elements](#accepted-syntax-of-circuit-elements)
- [UML Diagrams](#uml-diagrams)
- [Constraints](#constraints)
- [List of Errors and Warnings](#list-of-errors-and-warnings)
  - [Errors](#errors)
  - [Warnings](#warnings)
- [Credits](#credits)

## Working

This section briefly explains how the simulator is structured and how it handles both linear and nonlinear elements during DC and transient analyses.

### Core architecture

- Parser: reads a SPICE-like netlist, collects `.SUBCKT` definitions, expands hierarchical instances, tokenizes lines (preserving parenthesized groups), and constructs a flat list of `CircuitElement` instances. The parser understands `POLY` declarations for nonlinear capacitors and inductors and supports dependent sources that reference other element variables.
- Graph & traversal (Node/Edge): the parser's element list is converted into a node connectivity graph. Traversal of this graph is the mechanism used to stamp element contributions into the Modified Nodal Analysis (MNA) matrix efficiently.
- Stamping: each concrete element implements a `stamp()` method that adds its contribution to the MNA matrix and RHS. The simulator supports group-1 elements (pure nodal stamps) and group-2 elements (elements whose branch current is an explicit unknown in the system).
- Linear algebra: assembled matrices are converted to Eigen types for numerical solution. LU factorization is used, with small diagonal regularization attempted if the matrix is singular.

### Nonlinear models and transient support

- Nonlinear constitutive models: The simulator has a compact abstraction for nonlinear constitutive relations (`include/NonlinearModel.hpp`). Two header-only polynomial models are provided:
  - `PolynomialChargeModel` — charge q(u) and dq/du for charge-based (capacitor) models.
  - `PolynomialFluxModel` — flux phi(i) and dphi/di for flux-based (inductor) models.
  Coefficients are stored in ascending power order and evaluated with numerically robust techniques (Horner-style accumulation and clamping where appropriate).
- Companion linearization (trapezoidal rule): Nonlinear capacitors/inductors are handled by computing a per-timestep linear companion model (conductance / source equivalents) suitable for use inside a Newton-Raphson nonlinear solve. Elements expose `computeCompanion(h)` and `computeCompanionIter(h, x, indexMap)` hooks used by the transient driver to update linearized parameters around the current iterate.
- Transient driver: the simulator implements a fixed-step transient algorithm using the trapezoidal rule and Newton–Raphson at each timestep:
  - Per-iterate, elements update companion parameters using the current iterate.
  - The assembly produces a Jacobian-like MNA matrix `A_k` and residual `b_k`.
  - The linear system is solved (Eigen LU). If the factorization indicates singularity, the solver tries small diagonal regularizers to recover a solvable system.
  - Under-relaxation/backtracking: candidate Newton updates are relaxed (scaled) and tested for finiteness and residual improvement. The solver rejects iterates that produce non-finite values or gross growth beyond configured safety bounds.
  - Convergence checks use absolute/relative step norms and residual norms; solver options (iteration limits, damping, backtracks, and safety thresholds) are exposed via `SolverOptions`.
- Initialization: element dynamic states (capacitor voltages, inductor currents) may be initialized either from the DC operating point (computed by `computeOperatingPoint`) or set to zero if `zeroInit` is enabled in `SolverOptions`.
- Output and diagnostics:
  - A CSV trace (`transient.csv`) is produced containing time and unknown values for each saved step.
  - Diagnostics are written to a log file (default `diagnostics.log`) and can include per-element diagnostic dumps when verbose diagnostics are enabled.
  - The solver defends against runaway iterates using `maxState` and other runtime guards in `SolverOptions`.
- Numeric safeguards:
  - Polynomial models use clamping (large finite sentinel) and overflow checks to avoid Inf/NaN propagation.
  - The transient driver attempts small diagonal regularizers when LU decomposition fails and performs backtracking to recover from poor iterates.
  - The parser can optionally add very large-value "stability resistors" from non-ground nodes to ground (`RSTAB_<node>`) with value `STABILITY_RESISTOR_VALUE = 1e12` to improve matrix conditioning in degenerate DC cases.

### Developer-visible hooks and extension points

- New nonlinear models: implement `NonlinearModel` and override the relevant methods (`q`/`dqdu` for charge-based, `phi`/`dphidi` for flux-based).
- Element companion API: elements expose `computeCompanion(h)` and `updateStateFromSolution(x, indexMap)` to participate in transient linearization and state updates.
- Parser extensibility: subcircuit collection/expansion is implemented and the tokenization preserves parenthesized expressions; extending supported behavioral expressions is localized to the Parser/tokenizer.

### Design rationale 

- Use MNA + explicit group-2 unknowns to keep the stamp rules simple and local to element implementations.
- Use per-element companion linearizations to allow a Newton-based solve for stiff/nonlinear systems while keeping element code modular.
- Favor robust, defensive numerical strategies (clamping, regularization, backtracking) to increase solver stability on student/experimental netlists.

## Installation and Running SNU Spice

### Requirements

- CMake (required)
- G++ (required)
- Eigen (header-only dependency; the project includes an external copy under `lib/external/Eigen`)

> Note: CMake and G++ are mandatory for configuring and building the project. The build instructions below assume both are available and on your PATH.

### Installation (CMake)

Follow these steps to configure and build the project using CMake:

1. Clone the repository:

2. Create a build directory and run CMake to configure and build:
```bash
mkdir build
cd build
cmake -S .. -B .
cmake --build .
```

3. After a successful build the main executable will be available under `build/src` (or the path printed by CMake).

### Running

- Edit or create your netlist in `circuit.sns` (or point to another file via command-line).
- From the project root run the built executable, optionally passing a path to a netlist file:
```bash
./build/src/SNU_Spice [path/to/your_netlist.sns]
```
(Replace `SNU_Spice` with the actual executable name if different; when built with the provided CMake the executable typically appears under `build/src`.)

The simulator will print status messages and, for transient runs, produce `transient.csv` in the current working directory. Diagnostic logs are written to `diagnostics.log` by default.

### Generating Documentation

If Doxygen documentation is configured in the CMake setup, you can produce docs via the `documentation` build target:
```bash
mkdir -p build
cd build
cmake -S .. -B .
cmake --build . -t documentation
```

### Tests

After configuring and building the project with CMake, run the test suite using CTest from the build directory:

```bash
mkdir -p build
cd build
cmake -S .. -B .
cmake --build .
ctest --test-dir . --output-on-failure
```

Note: the tests do not currently cover all cases or code paths.


## Accepted Syntax of Circuit Elements

This simulator recognizes a SPICE-like netlist. The following summarizes the exact token forms accepted by the parser and the special keywords/directives it understands. Tokens are tokenized after converting input to uppercase (the parser is case-insensitive for keywords) and the parser preserves parenthesized groups. Node names and element names are treated as strings; ground must be named `0`.

Element line syntax (token order)
- Independent current source (I):  
  `I<name> <node+> <node-> <value> [G2]`  
  Example: `I1 1 0 0.001` or `I2 N1 N2 5m G2`  
  Behavior: injects current into RHS. Optional `G2` requests the element's current be exposed as an unknown.

- Independent voltage source (V):  
  `V<name> <node+> <node-> <value>`  
  Example: `V1 1 0 5`  
  Notes: Voltage sources are always Group-2 (their branch current is an unknown).

- Dependent current source (IC):  
  `IC<name> <node+> <node-> <gain> <V|I> <controllingElement>`  
  Example: `IC1 1 0 2 V VCTRL` or `IC2 N1 N2 0.5 I R1`  
  Meaning: current injected = gain * (control variable). `V` means voltage of the referenced element; `I` means current of the referenced element. The controlling element name must be present in the netlist (resolution is done after parsing). Cascading controlled sources are rejected.

- Dependent voltage source (VC):  
  `VC<name> <node+> <node-> <gain> <V|I> <controllingElement>`  
  Example: `VC1 2 0 10 V VCTRL`  
  Meaning: voltage = gain * (control variable). Same control semantics as IC. Controlled sources are Group-2 devices.

- Resistor (R):  
  `R<name> <node+> <node-> <value> [G2]`  
  Example: `R1 1 2 1k` or `R2 N1 N2 1000 G2`  
  Behavior: standard conductance stamp; optional `G2` makes the branch current an explicit unknown.

- Capacitor (C): linear or polynomial (nonlinear):  
  - Linear capacitor: `C<name> <node+> <node-> <value> [G2]`  
    Example: `C1 1 0 1u`  
  - Polynomial (nonlinear) capacitor: `C<name> <node+> <node-> <value> POLY <a0> <a1> <a2> ...`  
    Example: `C2 2 0 1u POLY 0 1e-6 1e-9`  
    Semantics: when `POLY` appears as the fifth token, tokens after `POLY` are polynomial coefficients (ascending order: a0 + a1*u + a2*u^2 + ...). `value` (token 4) is parsed as a numeric baseline and is required; if no `POLY` is present, the element is treated as a linear capacitor with capacitance equal to `value`. The parser constructs a `NonlinearModel` from the coefficients.

- Inductor (L): linear or polynomial (nonlinear):  
  - Linear inductor: `L<name> <node+> <node-> <value>`  
    Example: `L1 1 0 1e-3`  
  - Polynomial (nonlinear) inductor: `L<name> <node+> <node-> <value> POLY <b0> <b1> <b2> ...`  
    Example: `L2 1 2 1e-3 POLY 0 1e-3 2e-6`  
    Semantics: when `POLY` appears, subsequent tokens are flux polynomial coefficients (phi(i) = b0 + b1*i + b2*i^2 + ...). Without `POLY` the inductor is linear with inductance `value`.

Subcircuit and instance syntax
- Subcircuit definition:  
  `.SUBCKT <name> <port1> <port2> ...`  
  ...body lines...  
  `.ENDS`  
  The parser collects subcircuit bodies and expands them when instances are encountered.

- Subcircuit instance (hierarchical):  
  `X<instname> <net1> <net2> ... <subcktName>`  
  Example: `XU1 N1 N2 MYSUB`  
  During expansion the parser substitutes ports and mangles internal element names using an `:<instancePath>` suffix to avoid collisions.

Top-level directives
- Operating point (DC):  
  `.OP`  
  Detects an operating-point analysis request.

- Transient directive:  
  `.TRAN <Tstep> <Tstop>`  
  Example: `.TRAN 1e-6 1e-3`  
  `Tstep` and `Tstop` are parsed as numeric tokens (see numeric rules below) and must be positive with `Tstep <= Tstop`.

Numeric tokens and parsing rules
- Numeric tokens accept integer, decimal, and scientific (exponential) notations, e.g. `10`, `3.3e-6`, `1E3`.
- SPICE-style suffix multipliers are recognized (case-insensitive): `T` (1e12), `G` (1e9), `MEG` (1e6), `K` (1e3), `M` (1e-3), `U` (1e-6), `N` (1e-9), `P` (1e-12), `F` (1e-15). Examples: `1K` → 1000, `2.2U` → 2.2e-6.
- Parsing is strict: the mantissa must be a valid floating literal and the optional suffix must be one of the recognized multipliers. Malformed numbers are rejected with diagnostics.

General notes and parser expectations
- Element name prefixes determine type: `R` (resistor), `V` (voltage source), `I` (current source), `C` (capacitor), `L` (inductor), `VC` (dependent voltage source), `IC` (dependent current source). The parser distinguishes `V` vs `VC` and `I` vs `IC` by prefix checks.
- Ground node must be named `0`. At least one ground node must be present.
- Optional `G2` token (when accepted for an element) requests that the element's branch current be included as an explicit unknown in the MNA index map.
- Dependent sources require a controlling element name; resolution to the actual element pointer is performed after parsing. If a dependent source references another dependent source (cascading), the parser will reject or warn depending on the case.
- Parentheses in tokens (e.g., behavioral expressions) are preserved by the tokenizer; inline comments starting with `*` or `;` are supported (rest of line is ignored) but inline non-comment text with extra stray characters in numeric tokens will cause parse errors.
- The parser supports `POLY` only for capacitors (`C`) and inductors (`L`) in the form shown above.


## Constraints

Important constraints enforced by the parser and solver (concise):

- Ground must exist: the netlist must include the ground node named `0`.
- Dependent-source current controls require the referenced element to be in group‑2. The parser will promote a referenced element to group‑2 automatically when necessary.
- Cascading controlled sources are not allowed: a dependent source controlled by another dependent source will be rejected or flagged.
- POLY declarations (for `C` and `L`) must include at least one coefficient after the `POLY` keyword; missing or empty coefficient lists are treated as parse errors.
- Numeric tokens must be well-formed: mantissa must be a valid floating literal and any suffix must be a recognized SPICE multiplier (T, G, MEG, K, M, U, N, P, F). Malformed numbers are rejected.
- Element terminals must differ: the two node tokens for a two‑terminal element must not be identical.
- Element values that would be invalid (e.g., zero for resistors when rejected by the element parser) will be reported and the line treated as erroneous.
- Subcircuit definitions must be well-formed: `.SUBCKT <name> <ports...>` requires a matching `.ENDS` and unique `.SUBCKT` names; unknown subcircuit instances are errors.

Note: the parser mangles internal names during subcircuit instance expansion (`<elem>:<instancePath>`) to avoid collisions, but choosing unique top-level names is still recommended to avoid confusion in diagnostics.

## List of Errors and Warnings

This section lists the important parse/validation conditions that the parser/solver reports. Only the key, actionable diagnostics are shown here.

### Errors (fatal / line rejected)
- Unknown element after subcircuit expansion (unrecognized leading token / element type).
- Missing ground node (`0`) in the netlist.
- Malformed numeric literal or unknown/misplaced suffix (strict numeric parsing failure).
- Missing or empty `POLY` coefficient list when `POLY` is specified for `C` or `L`.
- Undefined subcircuit instance (an `X...` instance referring to a non-existent `.SUBCKT`).
- Referenced controlling element not present in the netlist (dependent source references a name that was not defined).
- Identical terminals for a two‑terminal element (nodeA == nodeB) — treated as an error and the element is omitted.
- Invalid element value (e.g., a required non-zero value parsed as zero or invalid).

### Warnings (non-fatal / informational)
- Dependent-source referenced element was promoted to group‑2 because its current is referenced.
- Multiple solver directives found (e.g., more than one `.TRAN` or `.OP`) — parser uses the first directive and warns.
- Cascading controlled sources (dependent source referencing another dependent source) — parser may emit a warning and reject or degrade the construct depending on the case.
- Missing optional group token (e.g., omitted `[G2]`) — default behavior is applied and a warning may be emitted in ambiguous cases.

If you need a complete list of exact diagnostic strings emitted by the parser, check `src/Parser/Parser.cpp` and the element `parse()` helpers under `src/Elements/` which contain the literal messages used for errors and warnings.

## Credits

I am thankful to the [Eigen](https://eigen.tuxfamily.org/) library team for developing and maintaining the library, because of which the matrix equation could be solved efficiently.
