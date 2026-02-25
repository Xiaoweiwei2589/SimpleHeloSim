# SimpleHeloSim — A Minimal Rotorcraft Flight Dynamics + Trim + Linearization Sandbox (MATLAB/Simulink)

A small, educational helicopter flight-dynamics project written in **MATLAB + Simulink**.

This repo implements:

* A **rigid-body 6-DOF** fuselage equations-of-motion (EOM)
* A **static / quasi-steady rotorcraft aerodynamic model** (main rotor + tail rotor + fuselage + horizontal tail)
* A **numerical trimmer** (find equilibrium/trim)
* A **numerical linearizer** (finite-difference A, B, C, D)
* A **Simulink** model for time-domain simulation

The theory background is based on two PDFs included in this repo:

* `P101. Review of Kinematics and Dynamics.pdf`
* `P102. Overview of Rotorcraft Modeling and Simulation.pdf`

> ⚠️ Scope warning (honest): this is not a high-fidelity helicopter model. It is intentionally simple (uniform inflow, quasi-steady flapping, basic fuselage drag). Treat it as a **learning tool / prototyping sandbox**, not a validated aircraft model.

---

## What you can do with this project

1. **Trim** the helicopter at a desired forward speed (hover-ish is handled by using a small nonzero speed)
2. **Simulate** time response in Simulink using the trimmed initial condition
3. **Linearize** the dynamics about the trim point to obtain **A, B, C, D** for controls / stability analysis

---

## Repository contents

### Core model

* `SimpleHelo.m`
  Main model wrapper. Calls aerodynamics then EOM.
* `SimpleAero.m`
  Static helicopter aero model producing total body-axis **forces and moments**
  [X Y Z L M N] including:

  * main rotor thrust + torque, quasi-steady flapping (wind-axis → hub)
  * tail rotor thrust + torque
  * fuselage drag with induced downwash correction
  * horizontal tail lift with induced downwash correction
* `eqnmot.m`
  Rigid-body 6-DOF EOM with Euler angles and inertial position states

### Trim + linearization

* `SimpleHeloInit.m`
  Example script to set parameters, request trim speed, run trim, compute linear model
* `trimmer.m`
  Newton-like trim iteration using pseudo-inverse of Jacobian from numerical linearization
* `linearize.m`
  Finite-difference linearization to compute A,B,C,D (central differences)

### Utilities

* `table_lookup.m`
  1D linear interpolation table lookup for downwash factor vs wake skew angle

### Simulink

* `SimpleHeloSim.slx`
  Simulink model for simulation
* `SimpleHeloSim.slxc`
  Simulink cache (can be ignored / not tracked)

---

## Model definition (important for users)

### State vector `x` (12×1)

In `SimpleHelo.m` / `eqnmot.m`:

[
x = [u\ v\ w\ p\ q\ r\ \phi\ \theta\ \psi\ X\ Y\ Z]^T
]

* Velocities: `u v w` (ft/s) in body axes
* Body rates: `p q r` (rad/s)
* Euler angles: `phi theta psi` (rad)
* Position: `X Y Z` (ft) inertial (with the sign convention used in `eqnmot.m`)

### Control vector `u` (4×1)

In `SimpleHelo.m` / `SimpleAero.m`:

[
u = [\theta_{1c}\ \theta_{1s}\ \theta_0\ \theta_{0tr}]^T
]

* `theta1c` : main rotor cyclic (one axis)
* `theta1s` : main rotor cyclic (orthogonal axis)
* `theta0` : main rotor collective
* `theta0tr` : tail rotor collective

**Inputs are in degrees** in the interface; they are converted to radians inside `SimpleAero.m`.

### Outputs `y` (4×1)

From `SimpleHelo.m` / `SimpleAero.m`:

[
y = [V_{tot},\ a_x,\ a_y,\ a_z]^T
]

* `Vtot = sqrt(u^2+v^2+w^2)`
* Accelerations are computed as force/mass in body axes:

  * `ax = X/mass`, `ay = Y/mass`, `az = Z/mass`

---

## Units

This model uses **English engineering units**:

* Length: **ft**
* Time: **s**
* Velocity: **ft/s**
* Force: **lbf**
* Moment: **ft·lbf**
* Mass: **slug**
* Angles: **rad** for states, **deg** for control inputs (converted internally)

Gravity is set as `g = 32.17 ft/s^2`.

---

## Requirements

* MATLAB (recommended: R2020b or newer)
* Simulink (for the `.slx` model)
* No special toolboxes required for the core scripts (basic MATLAB is enough)

---

## Quick start (trim + linearize in MATLAB)

1. Clone the repo and open MATLAB in the repo directory.
2. Run the initialization + trim script:

```matlab
run SimpleHeloInit.m
```

3. When prompted:

* Enter trimmed forward speed in **ft/s** (example: `0.01` for near-hover, `50` for forward flight)

The script will:

* define aircraft constants (roughly Bell 206L-like)
* build an initial guess
* call `trimmer(...)` to solve for trim state/control
* call `linearize(...)` to produce `A, B, C, D`

You should see iteration output like:

```
Iteration     Max. Error
1            ...
2            ...
...
Successful trim
```

### What variables you get at the end

After `SimpleHeloInit.m` finishes, your workspace contains:

* `x0` : trimmed state
* `u0` : trimmed control
* `x_dot` : state derivatives at trim (should be close to targets)
* `y` : outputs at trim
* `A, B, C, D` : linear state-space matrices about trim

---

## Run the Simulink simulation

1. Make sure you have already run:

```matlab
run SimpleHeloInit.m
```

so the workspace contains the trimmed `x0`, `u0`, and `constants`.

2. Open Simulink model:

```matlab
open_system('SimpleHeloSim.slx')
```

3. Click **Run**.

> Note: If the Simulink model expects specific variable names (like `x0`, `u0`, `constants`), keep them unchanged.

---

## How trim works (high-level)

`trimmer.m` solves for a vector of trim variables (subset of states + controls) such that selected targets go to desired values.

* Trim variables are defined by `constants.TRIMVARS`
* Trim targets are defined by `constants.TRIMTARG`

In `SimpleHeloInit.m`, the default setting trims **state derivatives only** (`TRIMTARG = 1:12`), and it sets desired inertial kinematics targets like desired `Xdot, Ydot, Zdot` by putting those values into the target vector.

The Jacobian is computed from `linearize.m`, then a pseudo-inverse update is applied.

---

## How linearization works

`linearize.m` uses **central finite differences** for better accuracy than forward differences:

[
\frac{\partial f}{\partial x_i} \approx \frac{f(x_i+\Delta)-f(x_i-\Delta)}{2\Delta}
]

This is why you see the pattern:

* add `+DELXLIN`
* then subtract `2*DELXLIN` (net goes to `-DELXLIN`)

Outputs are also linearized to form `C` and `D`.

---

## Things that are intentionally simplified

If you’re a student: you should **know what is missing**, because that determines what conclusions you’re allowed to draw.

This model omits (by design):

* dynamic inflow states (uses uniform inflow iteration)
* blade element / wake models beyond a simple induced velocity correction
* rotor/engine/actuator dynamics
* full 6-DOF aero angles usage (some attitudes/positions not used by the aero module)
* detailed fuselage aerodynamics (just flat-plate drag areas)
* comprehensive stability derivatives / lookup tables

So: great for learning trimming, coupling, linearization, and control prototyping — not for certification-level prediction.

---

## Suggested learning exercises (for aerospace students)

1. **Hover trim**: try `VXTRIM = 0.01` and examine `u0` (collectives/cyclics).
2. **Forward flight trim sweep**: loop over speeds and plot:

   * collective vs speed
   * pitch attitude vs speed
3. **Eigenanalysis**: compute eigenvalues of `A`:

   ```matlab
   eig(A)
   ```
4. **Control design toy problem**: design an LQR using `(A,B)` and test in Simulink.
5. **Improve the model**: add actuator first-order lags and compare response.

---

## References

This repo is based on concepts summarized in:

* `P101. Review of Kinematics and Dynamics.pdf`
* `P102. Overview of Rotorcraft Modeling and Simulation.pdf`

---

## License

This project is licensed under the MIT License - see the LICENSE.md file for details

---

## Citation

If you want academic-friendly usage, add a `CITATION.cff`. Example text:

> Zhang, G. “SimpleHeloSim: Minimal rotorcraft trim + simulation sandbox in MATLAB/Simulink”, GitHub repository, 2026.

---

## Contributing

PRs are welcome, especially for:

* documentation improvements
* additional trim cases (turning flight, climb)
* adding rotor/actuator dynamics
* automated tests (sanity checks for trim + linearization)

Please include:

* what changed
* why it’s correct
* how you tested it

---

## Contact

Open an Issue on GitHub for bugs/questions. If you fork and extend the model, consider linking your results (plots, test cases) in the PR description.

---

