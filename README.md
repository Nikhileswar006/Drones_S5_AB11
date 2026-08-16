# Drones_S5_AB11
Drones project 
# Team members
  Supreet CB.SC.U4AIE24139  
  rohit   CB.SC.U4AIE24145
  Nikhil  CB.SC.U4AIE24063
  Phanendhra  CB.SC.U4AIE24032
  Koushik CB.SC.U4AIE24167
  
# Quaternion-Based Attitude Tracking Control for Quadrotor UAV

## 1. Project Overview

This project implements a quaternion-based attitude tracking controller for a quadrotor UAV using the backstepping control method.

The main objective is to make the quadrotor accurately follow a desired attitude trajectory in three rotational directions:

- Roll
- Pitch
- Yaw

The project is based on the research paper:

**"Quaternion-Based Attitude Tracking Control Design for UAVs"**

The paper uses quaternions to represent the UAV attitude and applies a backstepping methodology with a fractional exponent to design the attitude tracking controller.

The controller is evaluated through numerical simulation.

---

## 2. Problem Statement

A quadrotor must continuously control its orientation while flying.

The orientation of a quadrotor can be represented using:

- Euler angles
- Rotation matrices
- Quaternions

Euler angles are intuitive, but they can suffer from gimbal lock. Therefore, this project uses quaternions to represent the attitude.

The main control problem is:

> Given a desired attitude trajectory, calculate the control torque required to make the actual quadrotor attitude follow that trajectory.

The project therefore focuses on the relationship:

    Desired Attitude
          ↓
    Attitude Error
          ↓
    Backstepping Controller
          ↓
    Control Torque
          ↓
    Quadrotor Dynamics
          ↓
    Actual Attitude
          ↓
       Feedback

---

# 3. Project Objectives

The main objectives are:

1. Model the rotational dynamics of a quadrotor UAV.
2. Represent the UAV attitude using unit quaternions.
3. Calculate the quaternion attitude error.
4. Calculate the angular velocity tracking error.
5. Design a virtual control input using backstepping.
6. Design the actual control torque.
7. Make the attitude tracking errors converge to zero.
8. Simulate the complete closed-loop system in MATLAB.
9. Plot and analyze the attitude and angular velocity tracking performance.

---

# 4. Why Quaternions?

A quaternion represents a 3D orientation using four components:

    q = [q0, q1, q2, q3]^T

Quaternions are used because they:

- Avoid gimbal lock.
- Represent 3D orientation using four parameters.
- Provide smooth attitude representation.
- Are suitable for nonlinear UAV attitude control.

The paper specifically uses quaternions as the fundamental representation for UAV attitude.

---

# 5. Why Backstepping?

The quadrotor cannot directly control its attitude using the control input.

The physical relationship is approximately:

    Torque
       ↓
    Angular Velocity
       ↓
    Attitude

The motors generate torque.

Torque changes angular velocity.

Angular velocity changes the attitude.

Therefore, the controller is designed step-by-step.

This is the basic idea of backstepping.

The controller first determines a desired angular velocity that would reduce the attitude error. It then designs the required torque to make the actual angular velocity follow this desired value.

---

# 6. Overall System Architecture

The complete project follows a closed-loop architecture:

    ┌──────────────────────────────┐
    │      Reference Generator     │
    │  Desired Angular Velocity    │
    │       / Attitude             │
    └──────────────┬───────────────┘
                   │
                   ▼
    ┌──────────────────────────────┐
    │   Quaternion Error Calculator │
    │                              │
    │ qerr = qref* ⊗ q             │
    └──────────────┬───────────────┘
                   │
                   ▼
    ┌──────────────────────────────┐
    │     Backstepping Controller   │
    │                              │
    │  Virtual Control ωerr,d      │
    │  Error δ                     │
    │  Torque τ                    │
    └──────────────┬───────────────┘
                   │
                   ▼
    ┌──────────────────────────────┐
    │      Quadrotor Dynamics       │
    │                              │
    │ Quaternion Kinematics        │
    │ Rotational Dynamics          │
    └──────────────┬───────────────┘
                   │
                   ▼
    ┌──────────────────────────────┐
    │       Actual UAV State        │
    │                              │
    │ q, ω                         │
    └──────────────┬───────────────┘
                   │
                   │ Feedback
                   └─────────────────────┐
                                         │
                                         ▼
                                  Error Calculation

---

# 7. Mathematical Model

## 7.1 Quaternion Representation

The attitude is represented by:

    q = [q0 q1 q2 q3]^T

where:

- q0 is the scalar component.
- q1, q2, q3 represent the vector components.

For a valid unit quaternion:

    ||q|| = 1

---

## 7.2 Quaternion Conjugate

The quaternion conjugate is:

    q* = [q0 -q1 -q2 -q3]^T

For a unit quaternion:

    q^-1 = q*

The conjugate is used to obtain the inverse rotation when calculating the attitude error.

---

## 7.3 Quaternion Error

The attitude error is defined as:

    qerr = qref* ⊗ q

where:

- qref = desired quaternion
- q = actual quaternion
- qref* = conjugate of desired quaternion
- ⊗ = quaternion multiplication

When the actual and desired attitudes are equal:

    q = qref

then:

    qerr = [1 0 0 0]^T

This represents zero attitude error.

---

# 8. Quadrotor Attitude Dynamics

The project uses two main equations to model the rotational motion of the UAV.

## 8.1 Quaternion Kinematics

The quaternion differential equation is:

    q_dot = 1/2 q ⊗ [0; ω]

where:

- q = attitude quaternion
- ω = angular velocity
- q_dot = rate of change of attitude

This equation describes how the UAV orientation changes according to its angular velocity.

---

## 8.2 Rotational Dynamics

The rotational dynamics are:

    ω_dot = J^-1 [τ - ω × (Jω)]

where:

- J = inertia matrix
- ω = angular velocity
- τ = control torque
- ω × (Jω) = gyroscopic term
- ω_dot = angular acceleration

This equation determines how the control torque changes the angular velocity.

---

# 9. Error Definitions

## 9.1 Quaternion Error

    qerr = qref* ⊗ q

The objective is:

    qerr → [1 0 0 0]^T

---

## 9.2 Angular Velocity Error

The angular velocity error is:

    ωerr = ω - ωref

where:

- ω = actual angular velocity
- ωref = reference angular velocity

The objective is:

    ωerr → 0

---

# 10. Backstepping Controller

The backstepping controller is designed in two main stages.

## Stage 1: Virtual Control

The quaternion error dynamics are:

    qerr_dot =
        1/2 qerr ⊗ [0; ωerr]

Instead of directly controlling the quaternion, the controller creates a desired angular velocity error:

    [0; ωerr,d]
        =
    -k1 qerr* ⊗ (qerr - I)

where:

    k1 > 0

This is called the virtual control input.

Its purpose is to generate an angular velocity target that causes the attitude error to decrease.

---

# 11. Stable Quaternion Error Dynamics

After substituting the virtual control into the quaternion error dynamics:

    qerr_dot =
        -k1/2 (qerr - I)

This means that the quaternion error is driven toward:

    qerr = I

where:

    I = [1 0 0 0]^T

---

# 12. Second Backstepping Error

The actual angular velocity error may not immediately equal the desired angular velocity error.

Therefore, a second error is defined:

    δ = ωerr - ωerr,d

where:

- ωerr = actual angular velocity error
- ωerr,d = desired angular velocity error
- δ = remaining backstepping error

The controller aims to achieve:

    δ → 0

---

# 13. Error Dynamics

Differentiating the angular velocity error:

    ωerr_dot = ω_dot - ωref_dot

The dynamics of δ become:

    δ_dot = ωerr_dot - ωerr,d_dot

Substituting the quadrotor rotational dynamics gives:

    δ_dot =
        J^-1 [τ - ω × (Jω)]
        - ωref_dot
        - ωerr,d_dot

This equation contains the control torque τ.

Therefore, it is used to design the final torque controller.

---

# 14. Control Torque Design

The total torque is divided into two parts:

    τ = τ1 + τ2

The two terms have different purposes.

---

## 14.1 Torque Cancellation Term

The first term is:

    τ1 =
        ω × (Jω)
        + Jωref_dot
        + Jωerr,d_dot

Purpose:

- Cancel the gyroscopic term.
- Cancel the reference angular acceleration.
- Cancel the desired angular velocity error acceleration.

Therefore, τ1 is mainly a nonlinear compensation/cancellation term.

---

## 14.2 Error Convergence Term

The second term is:

    τ2 = -k2 J δ^(1/3)

where:

    k2 > 0

Purpose:

- Reduce the remaining error δ.
- Drive δ toward zero.
- Provide the desired nonlinear convergence behavior.

---

# 15. Final Error Dynamics

Substituting τ1 and τ2 into the error dynamics gives:

    δ_dot = -k2 δ^(1/3)

This is the final closed-loop error equation.

The objective is:

    δ → 0

The paper uses the fractional exponent 1/3 as part of the controller design.

---

# 16. Simulation Architecture

The MATLAB simulation follows this sequence:

    Step 1
    Define UAV parameters
             ↓
    Step 2
    Generate reference trajectory
             ↓
    Step 3
    Generate qref and ωref
             ↓
    Step 4
    Calculate quaternion error
             ↓
    Step 5
    Calculate ωerr
             ↓
    Step 6
    Calculate virtual control ωerr,d
             ↓
    Step 7
    Calculate δ
             ↓
    Step 8
    Calculate ωerr,d_dot
             ↓
    Step 9
    Calculate τ1
             ↓
    Step 10
    Calculate τ2
             ↓
    Step 11
    Calculate total torque τ
             ↓
    Step 12
    Apply τ to UAV dynamics
             ↓
    Step 13
    Update ω and q
             ↓
    Step 14
    Feed actual state back to controller
             ↓
    Repeat for every simulation time step

---

# 17. Reference Input

This is a model-based control simulation.

No machine-learning dataset is required.

The desired trajectory is generated mathematically.

The paper states that the reference angular velocity consists of three sine waves and that the trajectory is defined by the researchers.

Therefore:

    Reference Generator
            ↓
        ωref(t)
            ↓
       Controller

The reference is the target that the UAV must track.

---

# 18. Simulation Parameters

The paper uses:

### Inertia Matrix

    J = diag[0.1, 0.1, 0.12]

### Controller Gain

    k1 = 20

### Controller Gain

    k2 = 2

The paper also specifies the initial reference angular velocity and quaternion, and the initial UAV angular velocity and quaternion.

Initial reference angular velocity:

    ωref(0) =
    [-0.1
     -0.2
      0.2]

Initial reference quaternion:

    qref(0) =
    [0.94628
     -0.1541
      0.19051
     -0.21098]

Initial UAV angular velocity:

    ω(0) =
    [-0.1
     -0.2
      0.2]

Initial UAV quaternion:

    q(0) =
    [0.94628
     -0.1541
      0.19051
     -0.21098]

These are the simulation initialization values reported in the paper.

---

# 19. Closed-Loop Simulation

The controller continuously receives:

    Desired state
          +
    Actual state

and calculates:

    qerr
    ωerr
    ωerr,d
    δ
    τ

The torque is then applied to the mathematical quadrotor model.

The updated state is:

    q
    ω

These states are fed back to the controller.

Therefore, the simulation is a closed-loop system.

---

# 20. Software Requirements

## Required Software

- MATLAB
- MATLAB ODE solvers or equivalent numerical integration
- Basic MATLAB plotting functions

## Recommended Development Environment

MATLAB is recommended because the project consists primarily of:

- Matrix calculations
- Quaternion operations
- Differential equations
- Nonlinear control equations
- Numerical simulation
- Result plotting

Simulink can also be used to represent the same controller using interconnected blocks.

---

# 21. Suggested Project Structure

DRONES_SIMULATION/
├── main.m                                # Master simulation script
├── parameters.m                          # Physical UAV constants & controller gains
├── create_simulink_model.m               # Script to build & open Simulink .slx model
├── Quaternion/                           # Quaternion algebra functions
│   ├── quatMultiply.m                   # Hamilton product q1 x q2
│   ├── quatConjugate.m                  # Quaternion conjugate q*
│   ├── quatNormalize.m                  # Unit normalization q / ||q||
│   └── quatInverse.m                    # Quaternion inverse q^-1
├── Controller/                           # Backstepping controller equations
│   ├── quaternionError.m                # Eq.(10): Attitude error quaternion q_err
│   ├── angularVelocityError.m           # Eq.(11): Angular velocity error w_err
│   ├── desiredOmegaError.m              # Eq.(13): Virtual control w_err_d
│   ├── deltaError.m                     # Eq.(16): Backstepping error delta
│   ├── desiredOmegaDerivative.m         # Eq.(15): Time derivative d(w_err_d)/dt
│   ├── torque1.m                        # Eq.(21): Feedback cancellation torque tau1
│   ├── torque2.m                        # Eq.(22): Stabilizing control torque tau2
│   └── totalTorque.m                    # Eq.(23): Total torque tau = tau1 + tau2
├── Dynamics/                             # Quadrotor physical dynamics
│   ├── quaternionDynamics.m             # Eq.(8): Kinematic derivative dq/dt
│   └── angularDynamics.m                # Eq.(9): Rotational derivative dw/dt
├── Simulation/                           # ODE integration & visualization
│   ├── simulate.m                       # 13-Step numerical simulation solver (ode45)
│   └── plotResults.m                    # Paper trajectory tracking figures (Fig 3-12)
├── Utils/                                # Coordinate transformations
│   ├── quaternionToEuler.m              # Quat to Roll-Pitch-Yaw (rad)
│   └── eulerToQuaternion.m              # Roll-Pitch-Yaw to Quat
└── README.md                            # Comprehensive project guide

---

# 22. Main Program Flow

The main MATLAB program should perform:

    Initialize parameters
            ↓
    Set initial conditions
            ↓
    Generate reference
            ↓
    Start numerical simulation
            ↓
    Controller calculates torque
            ↓
    UAV dynamics calculate state derivatives
            ↓
    Numerical solver updates states
            ↓
    Store simulation data
            ↓
    Plot results

---

# 23. Expected Outputs

The simulation should generate the following results:

### 1. Quaternion Error

The quaternion error should converge toward:

    [1, 0, 0, 0]

This indicates that the actual and desired attitudes have aligned.

### 2. Yaw Tracking

Actual yaw should follow the desired yaw trajectory.

### 3. Pitch Tracking

Actual pitch should follow the desired pitch trajectory.

### 4. Roll Tracking

Actual roll should follow the desired roll trajectory.

### 5. Angular Velocity Error

The angular velocity error should converge toward zero.

### 6. Angular Velocity Tracking

The actual roll, pitch, and yaw angular velocities should converge to their reference values.

### 7. δ Error

The backstepping error δ should converge toward zero.

### 8. Control Torque

The control torque should reduce as the system approaches equilibrium.

---

# 24. Result Interpretation

The paper reports that:

- The quaternion error converges to [1, 0, 0, 0].
- Angular velocity error converges to zero at approximately 0.8 seconds.
- δ converges to zero at approximately 0.4 seconds.
- The torque converges to zero when equilibrium is reached.

These results demonstrate that the proposed controller successfully tracks the desired attitude trajectory.

---

# 25. Why No Dataset Is Required

This project is not a machine-learning system.

It does not train a model using:

- Images
- Sensor datasets
- CSV datasets
- Training data
- Testing data

Instead, it uses a mathematical model of the quadrotor.

The simulation generates the reference trajectory mathematically and solves the UAV differential equations numerically.

Therefore:

    Mathematical Model
          +
    Reference Trajectory
          +
    Controller
          ↓
    Numerical Simulation
          ↓
    Tracking Results

---

# 26. Important Variables

| Symbol | Meaning |
|--------|---------|
| q | Actual quaternion |
| qref | Desired quaternion |
| qerr | Quaternion attitude error |
| q* | Quaternion conjugate |
| ω | Actual angular velocity |
| ωref | Reference angular velocity |
| ωerr | Angular velocity error |
| ωerr,d | Desired angular velocity error |
| δ | Backstepping error |
| J | UAV inertia matrix |
| τ | Total control torque |
| τ1 | Dynamics cancellation torque |
| τ2 | Error convergence torque |
| k1 | First controller gain |
| k2 | Second controller gain |

---

# 27. Key Concepts

The project combines four major concepts:

### 1. Quaternion Representation

Used to represent UAV orientation without Euler-angle singularities.

### 2. Quadrotor Rotational Dynamics

Used to describe how torque changes angular velocity and attitude.

### 3. Backstepping Control

Used to design the controller recursively from attitude error to angular velocity and finally to torque.

### 4. Fractional-Power Error Feedback

The term:

    δ^(1/3)

is used in the final error feedback law.

---

# 28. Complete Mathematical Flow

    q, ω
     │
     ▼
    qerr = qref* ⊗ q
     │
     ▼
    ωerr = ω - ωref
     │
     ▼
    ωerr,d
     │
     ▼
    δ = ωerr - ωerr,d
     │
     ▼
    τ1 = ω × Jω + Jωref_dot + Jωerr,d_dot
     │
     ▼
    τ2 = -k2 J δ^(1/3)
     │
     ▼
    τ = τ1 + τ2
     │
     ▼
    ω_dot = J^-1[τ - ω × Jω]
     │
     ▼
    q_dot = 1/2 q ⊗ [0;ω]
     │
     ▼
    q, ω
     │
     └──────────── Feedback ────────────┘

---

# 29. Project Workflow in Simple Words

The project can be summarized as:

1. Tell the simulated drone what attitude/angular-velocity trajectory it should follow.
2. Represent the desired and actual attitude using quaternions.
3. Calculate the difference between desired and actual attitude.
4. Convert this attitude error into a desired angular-velocity error.
5. Compare the desired angular-velocity error with the actual angular-velocity error.
6. Calculate the remaining error δ.
7. Design the torque required to remove this error.
8. Apply the torque to the mathematical UAV model.
9. Calculate the new angular velocity.
10. Calculate the new quaternion.
11. Feed the new state back to the controller.
12. Repeat until the simulation ends.
13. Plot the tracking performance.

---

# 30. Advantages of the Proposed Approach

- Quaternion-based attitude representation.
- Avoids gimbal-lock problems associated with Euler-angle representation.
- Nonlinear control approach.
- Backstepping provides a systematic controller design.
- Fractional-power feedback is used in the final error dynamics.
- Can be evaluated completely through numerical simulation.
- Does not require a machine-learning dataset.

---

# 31. Limitations of the Current Project

The current work is primarily a numerical simulation.

The paper's presented validation is simulation-based, and the authors identify experimental validation as future work.

Therefore, the current project does not necessarily include:

- Real quadrotor hardware
- Real motor drivers
- Real IMU sensors
- Real-time flight testing
- Wind/disturbance testing
- Hardware-in-the-loop testing

These can be considered future extensions.

---

# 32. Future Scope

Possible future improvements include:

1. Implement the controller on a real quadrotor.
2. Integrate IMU sensor measurements.
3. Test the controller under wind disturbances.
4. Add actuator/motor dynamics.
5. Compare backstepping with PID, LQR, or other nonlinear controllers.
6. Perform hardware-in-the-loop testing.
7. Extend attitude control to full position and trajectory control.
8. Analyze robustness under parameter uncertainty.

The paper itself states that experimental testing is planned as future work.

---

# 33. Conclusion

This project develops and simulates a quaternion-based backstepping attitude tracking controller for a quadrotor UAV.

The quadrotor is mathematically modeled using quaternion kinematics and rotational dynamics. The desired and actual attitudes are compared using quaternion error. A backstepping controller first generates a virtual desired angular-velocity error and then designs the actual control torque.

The total torque consists of:

    τ = τ1 + τ2

where τ1 compensates for known system dynamics and τ2 drives the remaining error toward zero.

The resulting closed-loop error dynamics are:

    δ_dot = -k2 δ^(1/3)

The simulation evaluates quaternion error, attitude tracking, angular velocity tracking, δ convergence, and control torque.

The goal is to demonstrate that the quadrotor can accurately track the desired attitude trajectory with the proposed nonlinear controller.
