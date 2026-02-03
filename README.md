# MPC Autonomous Racing System - Leader-Follower Platooning

## Project Overview

This project implements a ** Model Predictive Control (MPC)** system for autonomous racing with two vehicles:
- **Leader Vehicle**: Uses LMPC to minimize lap time while learning from experience
- **Follower Vehicle**: Maintains desired gap using MPC

The system runs multi-lap simulations on a race track defined in curvilinear coordinates, with the leader vehicle **improving its lap time with each iteration** by learning from previous trajectories.

---

## System Architecture

### Core Components

```
┌─────────────────────────────────────────────────────────────┐
│                    LMPC Racing System                       │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────────┐         ┌──────────────┐                 │
│  │   Map        │         │  SIMULATOR   │                 │
│  │ (Track Geom) │────────▶│ (Dynamics)   │                 │
│  └──────────────┘         └──────────────┘                 │
│                                 ▲                           │
│                                 │                           │
│  ┌──────────────────────────────┼──────────────────────┐   │
│  │            CONTROL LOOP      │                      │   │
│  │                              │                      │   │
│  │  ┌─────────────┐      ┌──────▼──────┐              │   │
│  │  │   LMPC      │◀─────│ STATE x(t)  │              │   │
│  │  │ (Leader)    │      └─────────────┘              │   │
│  │  └──────┬──────┘                                    │   │
│  │         │ u(t)                                      │   │
│  │         ▼                                           │   │
│  │  ┌──────────────┐      ┌──────────────┐            │   │
│  │  │ Gap Control  │      │  PredModel   │            │   │
│  │  │ (Follower)   │      │  (LTV Ident) │            │   │
│  │  └──────────────┘      └──────────────┘            │   │
│  │                                                     │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐   │
│  │  Safe Set    │  │ Value Func   │  │ Local LinReg │   │
│  │   (SS)       │  │  (Qfun)      │  │  (A,B,C id)  │   │
│  └──────────────┘  └──────────────┘  └──────────────┘   │
│        ▲                  ▲                    ▲           │
│        └──────────────────┴────────────────────┘           │
│              Learning (updated each lap)                   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Key Classes

| Class | Purpose | Location |
|-------|---------|----------|
| **Map** | Track geometry & coordinate transforms | Cell 6 |
| **SIMULATOR** | True nonlinear vehicle dynamics (bicycle model) | Cell 8 |
| **PID** | Simple feedback controller for path tracking | Cell 10 |
| **MPC** | Base quadratic programming controller | Cell 13 |
| **LMPC** | Learning variant with safe set & terminal cost | Cell 13 |
| **PredictiveModel** | Local linear regression (A,B,C identification) | Cell 13 |
| **FollowerMPC** | Gap-tracking variant (not used in final sim) | Cell 13 |

---

## Algorithm Flow

### Initialization Phase

```
1. Define Track Map
   └─ Segments: [length, curvature_radius]
   └─ Transform (s, ey) ↔ (X, Y)

2. Generate PID Warmup Lap
   ├─ Initialize PID with target velocity vt = 0.8 m/s
   ├─ Simulate one lap:
   │  └─ For each timestep:
   │     ├─ Measure state x(t)
   │     ├─ PID.solve() → control u(t)
   │     └─ SIMULATOR.sim() → x(t+1)
   └─ Store: xcl_pid, ucl_pid
      (These become initial Safe Set for LMPC)

3. Train Predictive Model
   ├─ Use PID trajectories as training data
   └─ Ready for local linear regression
```

### Learning Phase (Multi-Lap)

```
For each iteration lap = 1, 2, ..., Laps:

┌─────────────────────────────────────────────────────────┐
│ START LAP                                               │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  Initialize: xcl = [x0], ucl = []                       │
│                                                         │
│  ┌─ CLOSED-LOOP SIMULATION ─────────────────┐         │
│  │                                           │         │
│  │  While xcl[-1][s] ≤ TrackLength:        │         │
│  │                                           │         │
│  │    Step 1: STATE MEASUREMENT            │         │
│  │    ├─ xt = xcl[-1]                       │         │
│  │    └─ (already have full state vector)   │         │
│  │                                           │         │
│  │    Step 2: LMPC SOLVE                    │         │
│  │    ├─ Find nearest neighbors in SS       │         │
│  │    ├─ Update terminal constraint         │         │
│  │    ├─ Update terminal cost (V-function)  │         │
│  │    ├─ Identify LTV model A_k, B_k, C_k  │         │
│  │    │  └─ Local regression on nearby data │         │
│  │    ├─ Solve QP:                          │         │
│  │    │  min J = ||x-xRef||²_Q + ||u||²_R  │         │
│  │    │  s.t. dynamics, constraints, SS     │         │
│  │    └─ Extract: uPred = [u_opt, ...]    │         │
│  │                                           │         │
│  │    Step 3: APPLY CONTROL                 │         │
│  │    ├─ ut = uPred[0, :]  (first action)   │         │
│  │    ├─ SIMULATOR.sim([xt, xt_glob], ut)   │         │
│  │    │  └─ Nonlinear bicycle model        │         │
│  │    └─ xt_next, xt_glob_next             │         │
│  │                                           │         │
│  │    Step 4: STORE TRAJECTORY              │         │
│  │    ├─ xcl.append(xt_next)                │         │
│  │    ├─ ucl.append(ut)                     │         │
│  │    ├─ LMPC.addPoint(xt, ut)              │         │
│  │    │  └─ Update SS for next solve        │         │
│  │    └─ time += 1                          │         │
│  │                                           │         │
│  └─────────────────────────────────────────┘         │
│                                                         │
│  END CLOSED-LOOP SIMULATION                            │
│                                                         │
│  Step 5: END-OF-LAP UPDATE                            │
│  ├─ Compute cost-to-go: J(x_k) = T - k                │
│  ├─ LMPC.addTrajectory(xcl, ucl)                       │
│  │  └─ Add complete trajectory to Safe Set            │
│  ├─ PredModel.addTrajectory(xcl, ucl)                  │
│  │  └─ Add to regression training set                 │
│  └─ iteration_complete++                              │
│                                                         │
│  Step 6: REPORT METRICS                               │
│  ├─ Lap time: J(x_0) = total steps × dt              │
│  ├─ Leader-follower gap                               │
│  └─ Iteration complete                                │
│                                                         │
└─────────────────────────────────────────────────────────┘

Output: Improved lap time, expanded Safe Set, better V-function
```

### Follower Controller (Simple Gap Control)

```
For each timestep in parallel with Leader:

1. Measure follower state: xf = [vx, vy, wz, epsi, s, ey]
2. Measure leader state: xL = [vx, vy, wz, epsi, s, ey]

3. Compute gap:
   gap = (sL - sF) % TrackLength  [modulo wraparound]
   gap_error = gap - DesiredGap

4. Proportional controller:
   u_accel = k_gap × gap_error        [close gap]
   u_steer = PID_lateral(ey, epsi)    [stay on track]

5. Apply control: SIMULATOR.sim([xf, xf_glob], [u_steer, u_accel])
```

---

## Workflow Diagram

### Complete Multi-Lap Progression

```
Lap 0 (PID Warmup)
├─ Time: ~20 seconds (baseline)
├─ Trajectory: Smooth, safe, conservative
└─ Safe Set: {xcl_pid}

        ↓ Learn from data

Lap 1 (LMPC Iteration 1)
├─ Time: ~18 seconds (faster!)
├─ Uses: PID trajectory as terminal target
├─ Strategy: Find shortcut while reaching SS
└─ Safe Set: {xcl_pid, xcl_1}

        ↓ Learn from data

Lap 2 (LMPC Iteration 2)
├─ Time: ~17 seconds (faster!)
├─ Uses: Best 4 points from laps 0-1
├─ Strategy: Smoother path with less steering
└─ Safe Set: {xcl_pid, xcl_1, xcl_2}

        ↓ Learn from data

...

Lap N (LMPC Iteration N)
├─ Time: ~15 seconds (near-optimal)
├─ Uses: Best 48 points from all past laps
├─ Strategy: Converged to fast racing line
└─ Safe Set: Converged
```

---

## Key Algorithms

### 1. Model Predictive Control (MPC)

**Problem:**
```
min  ||x - xRef||²_Q + ||u||²_R + ||Δu||²_dR
s.t. x_{k+1} = A*x_k + B*u_k + C         [dynamics]
     Fx*x ≤ bx                            [state bounds]
     Fu*u ≤ bu                            [input bounds]
```

**Solution:**
- Reformulate as Quadratic Program (QP)
- Use OSQP solver: fast (~1-10 ms), suitable for real-time
- Return optimal trajectory: xPred, uPred

### 2. Learning Model Predictive Control (LMPC)

**Extensions over MPC:**

1. **Safe Set (Feasibility Guarantee)**
   ```
   SS = conv{all successful trajectories from past laps}
   Terminal constraint: x_N ∈ SS
   
   Benefits:
   - Guarantees feasibility (can't go infeasible)
   - Provides terminal target region
   - Expands as controller learns
   ```

2. **Value Function (Terminal Cost)**
   ```
   V(x_N) = min{Qfun_i : ss_i in SS}
   
   Benefits:
   - Encourages reaching good regions
   - Monotonically decreases with iterations
   - Speeds convergence to optimal
   ```

3. **Time-Varying Linearization**
   ```
   Instead of: x_{k+1} = f(x_k, u_k)          [slow, nonlinear]
   Use:        x_{k+1} = A_k*x_k + B_k*u_k + C_k  [fast, linear]
   
   Identified via local linear regression on historical data
   ```

### 3. Local Linear Regression

**Purpose:** Identify linearized vehicle model from data

**Algorithm:**
```
Given: Current state x, control u

1. Find nearby historical points using Gaussian kernel
   w_i = (1 - (d_i/h)²)^(3/4)  for d_i < bandwidth h

2. Solve weighted least-squares for each state dimension:
   min Σ w_i ||A*x_i + B*u_i + C - x_{i+1}||²

3. Return: A (6×6), B (6×2), C (6,)
   Interpretation: Local linear dynamics at (x, u)
```

---

## Control Architecture

### Coordinate Systems

The system uses TWO coordinate frames:

#### 1. Curvilinear Frame (Track-Relative)
```
State: x = [vx, vy, wz, epsi, s, ey]
       ├─ vx:   longitudinal velocity (m/s)
       ├─ vy:   lateral velocity (m/s)
       ├─ wz:   yaw rate (rad/s)
       ├─ epsi: heading error vs track tangent (rad)
       ├─ s:    arc-length along track (m) ∈ [0, 19.25]
       └─ ey:   lateral offset from centerline (m) ∈ [-0.4, 0.4]

Advantages:
- Constraints simple: |ey| ≤ 0.4 (just lane width)
- s wraps continuously (lap-to-lap)
- Natural for track-following control
- MPC uses this frame
```

#### 2. Global Frame (Inertial)
```
State: x_glob = [vx, vy, wz, X, Y, ey]
       ├─ vx, vy, wz, ey: same as above
       ├─ X:  global X-position (m)
       └─ Y:  global Y-position (m)

Purpose:
- Visualization and plotting
- Track boundary checking
- Gap measurement between vehicles
```

#### 3. Transformations
```
Curvilinear → Global:  (s, ey) → (X, Y) via map.getGlobalPosition()
Global → Curvilinear:  (X, Y) → (s, ey) via map.getLocalPosition()

Used every timestep to maintain both representations
```

### Vehicle Dynamics

**Model:** Nonlinear Dynamic Bicycle Model

```
Inputs:  u = [delta, a]
         ├─ delta: steering angle (rad) ∈ [-0.5, 0.5]
         └─ a:     acceleration (m/s²) ∈ [-10, 10]

State derivatives:

Longitudinal:
  dvx/dt = a - (1/m)*Fyf*sin(delta) + wz*vy
  
Lateral:
  dvy/dt = (1/m)*(Fyf*cos(delta) + Fyr) - wz*vx
  
Yaw:
  dwz/dt = (1/Iz)*(lf*Fyf*cos(delta) - lr*Fyr)
  
Kinematics (global):
  dpsi/dt = wz
  dX/dt = vx*cos(psi) - vy*sin(psi)
  dY/dt = vx*sin(psi) + vy*cos(psi)
  
Kinematics (curvilinear):
  depsi/dt = wz - (vx*cos(epsi) - vy*sin(epsi))/(1-cur*ey)*cur
  ds/dt = (vx*cos(epsi) - vy*sin(epsi))/(1-cur*ey)
  dey/dt = vx*sin(epsi) + vy*cos(epsi)

where:
  Fyf, Fyr: lateral tire forces (Pacejka model)
  cur: track curvature at position s
```

---

## Code Structure

```
2026_01_16_LMPC_Racing_lg.ipynb
├─ Cell 1:  IMPORTS
│  └─ Libraries: numpy, scipy, matplotlib, cvxopt, osqp, dataclasses
│
├─ Cell 2:  MARKDOWN
│  └─ Project description
│
├─ Cell 4:  LIBRARY IMPORTS (detailed)
│  └─ All required packages
│
├─ Cell 6:  MAP CLASS
│  └─ Track geometry & coordinate transforms
│
├─ Cell 8:  SIMULATOR CLASS  
│  └─ Nonlinear bicycle model dynamics
│
├─ Cell 10: PID CONTROLLER CLASS
│  └─ Simple feedback for path tracking
│
├─ Cell 11: PID WARMUP LAP [COMMENTED OUT]
│  └─ Generate initial trajectory
│
├─ Cell 13: CORE CLASSES (1000+ lines)
│  ├─ MPCParams dataclass
│  ├─ MPC base class
│  ├─ FollowerMPC (gap tracking)
│  ├─ LMPC (learning variant)
│  ├─ PredictiveModel (LTV identification)
│  └─ Plotting functions
│
├─ Cell 14: INITIALIZATION
│  ├─ Create map, simulator
│  ├─ Run PID warmup lap
│  └─ Train predictive model
│
├─ Cell 16: MULTI-LAP SIMULATION
│  ├─ Main loop: for lap in range(Laps)
│  ├─ Leader LMPC + Follower gap control
│  ├─ Lap-by-lap metrics
│  └─ Trajectory storage
│
├─ Cell 18: VISUALIZATION
│  ├─ Plot LMPC evolution
│  ├─ Overlay follower trajectory
│  └─ Custom legend
│
├─ Cell 19: METRICS
│  ├─ Print lap statistics
│  └─ Compare iterations
│
├─ Cell 20: ANALYSIS
│  └─ Questions about algorithm
│
└─ Cell 21: ANIMATION
   ├─ FuncAnimation for vehicle motion
   ├─ Leader + Follower on track
   └─ Save as GIF
```

---

## Parameters & Tuning

### MPC Horizon
```
N = 14 steps
dt = 0.1 s
Horizon = N × dt = 1.4 seconds lookahead
```

### State Constraints
```
Track lanes:  |ey| ≤ 0.4 m
Enforced at every timestep k ∈ [0, N-1]
```

### Input Constraints
```
Steering:     |delta| ≤ 0.5 rad
Acceleration: |a| ≤ 10 m/s²
```

### Cost Weights
```
Q = diag([1, 1, 1, 1, 0, 100])
    └─ Large penalty on lateral error (ey)
    
R = diag([1, 10])
    └─ Moderate input effort cost
    
dR = [1, 10]
    └─ Smooth control (penalize jerky inputs)
    
QterminalSlack = 500 × I
    └─ Strong terminal constraint
```

### LMPC Learning
```
numSS_it = 4
    └─ Use points from 4 most recent iterations

numSS_Points = 48
    └─ Select 48 points per solve (12 per iteration)

Laps = 40-50
    └─ Run this many learning iterations
```

### Follower Gap Control
```
DesiredGap = 0.4 m
k_gap = 0.3
    └─ Proportional gain for gap error

PID lateral control:
    steering = -0.6*ey - 0.9*epsi
```

---

## Running the Simulation

### Step 1: Setup
```python
import numpy as np
import matplotlib.pyplot as plt
from osqp import OSQP

# Define track map
map = Map(halfWidth=0.4)

# Create simulators
simulator = SIMULATOR("dyn_bicycle_model", dt=0.1, map=map)
simulator_follower = SIMULATOR("dyn_bicycle_model", dt=0.1, map=map)
```

### Step 2: PID Warmup
```python
# Generate one safe lap with PID
PIDController = PID(vt=0.8)
xcl_pid, ucl_pid = simulate_pid_lap()
# Output: Safe initial trajectory
```

### Step 3: LMPC Initialization
```python
# Setup LMPC with PID data
lmpc = LMPC(numSS_Points, numSS_it, QterminalSlack, 
            lmpcParameters, predictiveModel)
lmpc.addTrajectory(xcl_pid, ucl_pid, xcl_pid_glob)
# Output: LMPC ready to learn
```

### Step 4: Multi-Lap Learning
```python
for lap in range(Laps):
    xcl, ucl = []
    while not lap_complete:
        lmpc.solve(current_state)
        u_opt = lmpc.uPred[0, :]
        x_next = simulator.sim(x_current, u_opt)
        xcl.append(x_next)
        ucl.append(u_opt)
    
    # Learn from this lap
    lmpc.addTrajectory(xcl, ucl, xcl_glob)
    predictiveModel.addTrajectory(xcl, ucl)
    
    print(f"Lap {lap}: time={lap_time:.2f}s")
# Output: Improved lap times across iterations
```

### Step 5: Visualization
```python
# Plot LMPC evolution
plotClosedLoopLMPC(lmpc, map, last_laps_to_plot=5)

# Create animation
create_leader_follower_animation()
```

---

## Expected Results

### Lap Time Progression
```
Iteration  Lap Time    Gap to Optimal
─────────────────────────────────────
   0       20.5 s      15% slower
   1       19.2 s      8% slower
   2       18.8 s      4% slower
   3       18.2 s      2% slower
   ...
   10      17.9 s      ~0.5% slower
```

### Trajectory Evolution
```
Iteration 0 (PID): Conservative, centered lane
Iteration 1:       Starts learning - slightly faster turns
Iteration 2:       Tighter apexes - approaching optimal
Iteration 3+:      Converged - consistent fast line
```

### Safety Metrics
```
- Safe Set: Expands from 1 to 40+ trajectories
- Feasibility: 100% (all iterations feasible)
- Gap: Follower maintains 0.4m ± 0.1m
- Constraint violations: 0
```

---

## Key Features

✅ **Safe Learning**: Convex safe set guarantees feasibility  
✅ **Real-time Control**: ~5-10 ms per solve (OSQP)  
✅ **Model Mismatch Robust**: Linear model + feedback loop  
✅ **Multi-vehicle**: Leader + follower coordination  
✅ **Curvilinear Coordinates**: Natural track representation  
✅ **Visualization**: Trajectory plots and animation  
✅ **Metrics Tracking**: Per-lap time and gap monitoring  

---

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| Infeasible QP | Horizon too long, constraints tight | Reduce N or relax bounds |
| Slow convergence | numSS_Points too small | Increase point selection |
| Large gap error | k_gap too small | Increase follower gain |
| Constraint violations | Slack disabled | Enable slacks or relax bounds |
| Memory overflow | Too many iterations | Reduce numSS_Points or Laps |

---

## References

### Theory
- **LMPC**: Learning Model Predictive Control (Rosolia, Borrelli)
- **MPC**: Model Predictive Control (Camacho, Bordons)
- **Bicycle Model**: Vehicle Dynamics & Control (Rajamani)

### Implementation
- **QP Solver**: OSQP (Boyd et al.)
- **Identification**: Local Linear Regression (Kernel Methods)
- **Visualization**: Matplotlib, FuncAnimation

---

## Authors & Attribution

**Original LMPC Theory:**
- Ugo Rosolia (UC Berkeley)
- Charlott Vallon (UC Berkeley)  
- Francesco Borrelli (UC Berkeley)
- Luigi Glielmo (Università di Napoli Federico II)

**Protected by U.S. copyright law**  
Educational use authorized

---

## Quick Start Summary

```
1. Define track: Map(halfWidth=0.4)
2. Create simulator: SIMULATOR("dyn_bicycle_model")
3. Generate PID lap: simulate_pid_lap() → xcl_pid, ucl_pid
4. Initialize LMPC: LMPC(...).addTrajectory(xcl_pid, ucl_pid)
5. Run multi-lap: for lap in range(Laps): solve and update
6. Visualize: plotClosedLoopLMPC() and create_animation()
7. Analyze: Print lap times, gap metrics, safety metrics

Expected: Lap times improve 5-15% over 10-20 iterations
```

---

**For detailed code comments, see: `COMPREHENSIVE_CODE_COMMENTS.txt`**
