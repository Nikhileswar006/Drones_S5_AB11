<p align="center">
  <img src="Amrita Vishwa Vidhyapeetam" width="200">
</p>

# Quaternion-Based Attitude Tracking Control for a Quadrotor UAV

## Team Members

- Supreeth — CB.SC.U4AIE24139
- Rohit — CB.SC.U4AIE24145
- Nikhil — CB.SC.U4AIE24063
- Phanendhra — CB.SC.U4AIE24032
- Koushik — CB.SC.U4AIE24167

---

## 1. Project Overview

This project focuses on attitude tracking control of a quadrotor UAV using quaternions and a backstepping control method.

The main objective is to make the quadrotor follow a desired attitude trajectory in roll, pitch, and yaw.

The controller is based on the research paper:

**"Quaternion-Based Attitude Tracking Control Design for UAVs"**

The paper uses quaternions to represent UAV attitude and applies a backstepping methodology with a fractional exponent. The proposed controller is evaluated using numerical simulation.

---

## 2. Problem Statement

A quadrotor needs to continuously control its orientation while flying.

The orientation of a quadrotor can be represented using:

- Euler angles
- Rotation matrices
- Quaternions

Euler angles are easy to understand, but they can suffer from gimbal lock. Because of this, quaternions are used in this project for representing the UAV attitude.

The main control problem is:

> Given a desired attitude trajectory, calculate the control torque required to make the actual quadrotor attitude follow that trajectory.

The overall control process is:

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

## 3. Project Objectives

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

## 4. Why Quaternions?

A quaternion represents a 3D orientation using four components.

The quaternion is written as:

    q = [q₀, q₁, q₂, q₃]ᵀ

It can also be written as:

    q = q₀ + i q₁ + j q₂ + k q₃

where:

- q₀ is the scalar component.
- q₁, q₂, q₃ are the vector components.

For a unit quaternion:

    ||q|| = √(q₀² + q₁² + q₂² + q₃²) = 1

Quaternions are useful because they:

- Avoid gimbal lock.
- Represent 3D orientation using four parameters.
- Provide a smooth attitude representation.
- Are suitable for nonlinear UAV attitude control.

---

## 5. Why Backstepping?

The quadrotor does not directly change its attitude using the control input.

The relationship is:

    Torque
       ↓
    Angular Velocity
       ↓
    Attitude

The motors generate torque.

The torque changes the angular velocity, and the angular velocity changes the attitude.

Because of this structure, the controller is designed step-by-step. This is the basic idea behind backstepping control.

The controller first determines a desired angular velocity that can reduce the attitude error. It then calculates the required torque to make the actual angular velocity follow this desired value.

---

## 6. Overall System Architecture

The complete project follows a closed-loop architecture:

    ┌──────────────────────────────┐
    │      Reference Generator     │
    │                              │
    │ Desired Angular Velocity     │
    │ / Attitude                   │
    └──────────────┬───────────────┘
                   │
                   ▼
    ┌──────────────────────────────┐
    │   Quaternion Error Calculator│
    │                              │
    │ qₑᵣᵣ = qᵣₑ𝒻* ⊗ q              │
    └──────────────┬───────────────┘
                   │
                   ▼
    ┌──────────────────────────────┐
    │     Backstepping Controller  │
    │                              │
    │ Virtual Control              │
    │ Error δ                      │
    │ Torque τ                     │
    └──────────────┬───────────────┘
                   │
                   ▼
    ┌──────────────────────────────┐
    │      Quadrotor Dynamics      │
    │                              │
    │ Quaternion Kinematics        │
    │ Rotational Dynamics          │
    └──────────────┬───────────────┘
                   │
                   ▼
    ┌──────────────────────────────┐
    │       Actual UAV State       │
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

## 7. Mathematical Model

### 7.1 Quaternion Representation

The attitude is represented by:

    q = [q₀, q₁, q₂, q₃]ᵀ

where:

- q₀ is the scalar component.
- q₁, q₂, q₃ are the vector components.

For a valid unit quaternion:

    ||q|| = 1

---

### 7.2 Quaternion Conjugate

The quaternion conjugate is:

    q* = [q₀, -q₁, -q₂, -q₃]ᵀ

The general quaternion inverse is:

    q⁻¹ = q* / ||q||²

For a unit quaternion:

    q⁻¹ = q*

The conjugate is used when calculating the attitude error.

---

### 7.3 Quaternion Normalization

A quaternion can be normalized using:

    qₙₒᵣₘ = q / ||q||

Normalization is useful in numerical simulation to keep the quaternion close to unit length.

---

### 7.4 Quaternion Multiplication

Quaternion multiplication is written as:

    q₁ ⊗ q₂

For:

    q₁ = [a, b, c, d]ᵀ
    q₂ = [e, f, g, h]ᵀ

the product is:

    q₁ ⊗ q₂ =
    [
    ae - bf - cg - dh
    af + be + ch - dg
    ag - bh + ce + df
    ah + bg - cf + de
    ]

Quaternion multiplication is non-commutative:

    q₁ ⊗ q₂ ≠ q₂ ⊗ q₁

Therefore, the order of quaternion multiplication must be maintained correctly.

---

## 8. Quadrotor Attitude Dynamics

The project uses two main equations to describe the rotational motion of the UAV.

### 8.1 Quaternion Kinematics

The quaternion differential equation is:

    q̇ = 1/2 q ⊗ [0; ω]

where:

- q = attitude quaternion
- ω = angular velocity
- q̇ = rate of change of attitude

This equation describes how the UAV orientation changes according to its angular velocity.

---

### 8.2 Rotational Dynamics

The rotational dynamics are:

    ω̇ = J⁻¹ [τ - ω × (Jω)]

where:

- J = inertia matrix
- ω = angular velocity
- τ = control torque
- ω × (Jω) = gyroscopic term
- ω̇ = angular acceleration

This equation determines how the control torque changes the angular velocity.

---

## 9. Error Definitions

### 9.1 Quaternion Error

The attitude error is defined as:

    qₑᵣᵣ = qᵣₑ𝒻* ⊗ q

where:

- qᵣₑ𝒻 = desired quaternion
- q = actual quaternion
- qᵣₑ𝒻* = conjugate of the desired quaternion
- ⊗ = quaternion multiplication

When the actual and desired attitudes are equal:

    q = qᵣₑ𝒻

then:

    qₑᵣᵣ = [1, 0, 0, 0]ᵀ

This represents zero attitude error.

The identity quaternion is:

    qᵢ = [1, 0, 0, 0]ᵀ

---

### 9.2 Angular Velocity Error

The angular velocity tracking error is:

    ωₑᵣᵣ = ω - ωᵣₑ𝒻

where:

- ω = actual angular velocity
- ωᵣₑ𝒻 = reference angular velocity

The control objective is:

    ωₑᵣᵣ → 0

---

## 10. Quaternion Error Dynamics

The quaternion error dynamics are:

    q̇ₑᵣᵣ = 1/2 qₑᵣᵣ ⊗ [0; ωₑᵣᵣ]

This equation connects the attitude error with the angular velocity tracking error.

The controller first creates a desired angular velocity error, which is then used in the second backstepping stage to generate the control torque.

---

## 11. Backstepping Controller

The backstepping controller is designed in two main stages.

### Stage 1: Virtual Control

A virtual control is designed using the quaternion error.

### Stage 2: Control Torque

The difference between the actual angular velocity error and the virtual control is used to design the final control torque.

The second error variable is:

    δ = ωₑᵣᵣ - ωₑᵣᵣ,𝒹

where:

- ωₑᵣᵣ = angular velocity error
- ωₑᵣᵣ,𝒹 = desired angular velocity error
- δ = backstepping error

---

## 12. Stage 1: Virtual Control

The virtual control is defined as:

    [0; ωₑᵣᵣ,𝒹] =
    -k₁ qₑᵣᵣ* ⊗
    (qₑᵣᵣ - qᵢ)

where:

- k₁ > 0
- qᵢ = [1, 0, 0, 0]ᵀ

This virtual control is designed to make the quaternion error converge toward the identity quaternion.

---

## 13. Stable Quaternion Error Dynamics

After substituting the virtual control into the quaternion error dynamics, the desired closed-loop behavior becomes:

    q̇ₑᵣᵣ =
    -(k₁/2)(qₑᵣᵣ - qᵢ)

This means that the quaternion error is driven toward:

    qₑᵣᵣ = qᵢ

or:

    qₑᵣᵣ = [1, 0, 0, 0]ᵀ

---

## 14. Stage 2: Backstepping Error

The second error variable is:

    δ = ωₑᵣᵣ - ωₑᵣᵣ,𝒹

where:

    ωₑᵣᵣ = ω - ωᵣₑ𝒻

and:

    ωₑᵣᵣ,𝒹 = desired angular velocity error

The control torque is designed to make:

    δ → 0

---

## 15. Error Dynamics

Starting from:

    δ = ωₑᵣᵣ - ωₑᵣᵣ,𝒹

differentiate both sides:

    δ̇ = ω̇ₑᵣᵣ - ω̇ₑᵣᵣ,𝒹

Since:

    ωₑᵣᵣ = ω - ωᵣₑ𝒻

we have:

    ω̇ₑᵣᵣ = ω̇ - ω̇ᵣₑ𝒻

Using the quadrotor rotational dynamics:

    ω̇ = J⁻¹ [τ - ω × (Jω)]

therefore:

    δ̇ =
    J⁻¹ [τ - ω × (Jω)]
    - ω̇ᵣₑ𝒻
    - ω̇ₑᵣᵣ,𝒹

This equation is used to design the final control torque.

---

## 16. Control Torque Design

The control torque is divided into two terms:

    τ = τ₁ + τ₂

where:

- τ₁ compensates for the system dynamics and reference motion.
- τ₂ provides error convergence.

---

## 17. Torque Term 1: Dynamics Compensation

The first torque term is:

    τ₁ =
    ω × (Jω)
    + Jω̇ᵣₑ𝒻
    + Jω̇ₑᵣᵣ,𝒹

This term compensates for:

- The nonlinear rotational dynamics.
- The reference angular velocity derivative.
- The derivative of the virtual control.

The derivative of the virtual control is obtained from:

    [0; ωₑᵣᵣ,𝒹] =
    -k₁ qₑᵣᵣ* ⊗
    (qₑᵣᵣ - qᵢ)

Therefore:

    [0; ω̇ₑᵣᵣ,𝒹] =
    d/dt {
    -k₁ qₑᵣᵣ* ⊗
    (qₑᵣᵣ - qᵢ)
    }

---

## 18. Torque Term 2: Error Convergence

The second torque term is:

    τ₂ = -k₂ Jδ¹ᐟ³

where:

    k₂ > 0

This fractional-power feedback term is used to drive the backstepping error toward zero.

The resulting error dynamics become:

    δ̇ = -k₂δ¹ᐟ³

---

## 19. Final Control Torque

Combining the two torque terms:

    τ = τ₁ + τ₂

Therefore:

    τ =
    ω × (Jω)
    + Jω̇ᵣₑ𝒻
    + Jω̇ₑᵣᵣ,𝒹
    - k₂Jδ¹ᐟ³

This is the main control law used in the simulation.

---

## 20. Final Error Dynamics

After substituting the control torque into the error dynamics:

    δ̇ = -k₂δ¹ᐟ³

The controller therefore drives:

    δ → 0

As the backstepping error converges, the angular velocity tracking error also converges toward the desired behavior.

---

## 21. Reference Trajectory

The reference angular velocity consists of three sinusoidal signals.

The paper describes the reference trajectory as being arbitrarily defined using sine waves.

The reference signal is represented as:

    ωᵣₑ𝒻 =
    [ωᵣₑ𝒻,₁, ωᵣₑ𝒻,₂, ωᵣₑ𝒻,₃]ᵀ

Its derivative is:

    ω̇ᵣₑ𝒻 = d(ωᵣₑ𝒻)/dt

The exact amplitudes and frequencies should be taken from the MATLAB implementation when running the simulation.

---

## 22. Simulation Parameters

The parameters used in the paper are:

| Parameter | Value |
|---|---|
| J | diag(0.1, 0.1, 0.12) |
| k₁ | 20 |
| k₂ | 2 |

Initial angular velocity:

    ω(0) = [-0.1, -0.2, 0.2]ᵀ

Initial reference angular velocity:

    ωᵣₑ𝒻(0) = [-0.1, -0.2, 0.2]ᵀ

Initial quaternion:

    q(0) = [0.94628, -0.1541, 0.19051, -0.21098]ᵀ

Initial reference quaternion:

    qᵣₑ𝒻(0) =
    [0.94628, -0.1541, 0.19051, -0.21098]ᵀ

---

## 23. Closed-Loop Simulation

The complete system is simulated as a closed-loop system.

The reference trajectory is given to the controller.

The controller calculates:

    qₑᵣᵣ

then:

    ωₑᵣᵣ

then:

    ωₑᵣᵣ,𝒹

then:

    δ

and finally:

    τ

The calculated torque is applied to the quadrotor rotational dynamics.

The updated angular velocity is then used to update the quaternion.

This process continues at every simulation step.

---

## 24. Simulation Procedure

The basic simulation loop is:

    1. Generate the reference trajectory.
    2. Read the current quaternion q.
    3. Read the current angular velocity ω.
    4. Calculate qᵣₑ𝒻*.
    5. Calculate qₑᵣᵣ = qᵣₑ𝒻* ⊗ q.
    6. Calculate ωₑᵣᵣ = ω - ωᵣₑ𝒻.
    7. Calculate the virtual control ωₑᵣᵣ,𝒹.
    8. Calculate δ = ωₑᵣᵣ - ωₑᵣᵣ,𝒹.
    9. Calculate ω̇ₑᵣᵣ,𝒹.
    10. Calculate τ₁.
    11. Calculate τ₂.
    12. Calculate the total torque τ.
    13. Update angular velocity using the rotational dynamics.
    14. Update quaternion using quaternion kinematics.
    15. Normalize the quaternion.
    16. Store the results.
    17. Plot the results.

---

## 25. Expected Simulation Results

### Quaternion Error

The quaternion error should converge toward:

    qₑᵣᵣ → [1, 0, 0, 0]ᵀ

### Angular Velocity Error

The angular velocity error should converge toward:

    ωₑᵣᵣ → [0, 0, 0]ᵀ

### Backstepping Error

The error:

    δ = ωₑᵣᵣ - ωₑᵣᵣ,𝒹

should converge toward:

    δ → [0, 0, 0]ᵀ

### Control Torque

The control torque should decrease as the system approaches the desired trajectory.

---

## 26. Results Reported in the Research Paper

The research paper reports that:

- The quaternion error converges toward [1, 0, 0, 0].
- The angular velocity error approaches zero at approximately 0.8 seconds.
- The backstepping error δ approaches zero at approximately 0.4 seconds.
- The control torque approaches zero when the system reaches equilibrium.

These results indicate that the proposed controller is able to track the desired attitude trajectory in simulation.

---

## 27. MATLAB Implementation

The MATLAB implementation can be organized using functions for quaternion operations, controller calculations, and system dynamics.

Important quaternion functions include:

    quatMultiply
    quatConjugate
    quatNormalize
    quatInverse

The controller calculates:

    qₑᵣᵣ
    ωₑᵣᵣ
    ωₑᵣᵣ,𝒹
    δ
    τ₁
    τ₂
    τ

The dynamics function calculates:

    q̇
    ω̇

---

## 28. Suggested Project Structure

A simple MATLAB project structure is:

    Drones_S5_AB11/
    │
    ├── README.md
    ├── main.m
    ├── controller.m
    ├── dynamics.m
    ├── referenceTrajectory.m
    ├── quatMultiply.m
    ├── quatConjugate.m
    ├── quatNormalize.m
    ├── quatInverse.m
    ├── plots.m
    │
    └── results/
        ├── quaternion_error.png
        ├── angular_velocity_error.png
        ├── delta.png
        └── control_torque.png

The exact file structure can be changed depending on the MATLAB implementation.

---

## 29. Main Program Flow

The main program follows this sequence:

    Start
      ↓
    Initialize Parameters
      ↓
    Initialize q and ω
      ↓
    Generate Reference Trajectory
      ↓
    Calculate Quaternion Error
      ↓
    Calculate Angular Velocity Error
      ↓
    Calculate Virtual Control
      ↓
    Calculate δ
      ↓
    Calculate Control Torque
      ↓
    Update Quadrotor Dynamics
      ↓
    Update Quaternion
      ↓
    Normalize Quaternion
      ↓
    Store Results
      ↓
    Plot Results
      ↓
    End

---

## 30. Important Variables

| Variable | Meaning |
|---|---|
| q | Actual quaternion |
| qᵣₑ𝒻 | Reference quaternion |
| qₑᵣᵣ | Quaternion attitude error |
| qᵢ | Identity quaternion |
| ω | Actual angular velocity |
| ωᵣₑ𝒻 | Reference angular velocity |
| ωₑᵣᵣ | Angular velocity error |
| ωₑᵣᵣ,𝒹 | Desired angular velocity error |
| δ | Backstepping error |
| J | Inertia matrix |
| τ | Total control torque |
| τ₁ | Dynamics compensation torque |
| τ₂ | Error convergence torque |
| k₁ | First controller gain |
| k₂ | Second controller gain |

---

## 31. Complete Mathematical Flow

The complete controller can be summarized as follows.

### Step 1: Quaternion Kinematics

    q̇ = 1/2 q ⊗ [0; ω]

### Step 2: Rotational Dynamics

    ω̇ = J⁻¹ [τ - ω × (Jω)]

### Step 3: Quaternion Error

    qₑᵣᵣ = qᵣₑ𝒻* ⊗ q

### Step 4: Angular Velocity Error

    ωₑᵣᵣ = ω - ωᵣₑ𝒻

### Step 5: Quaternion Error Dynamics

    q̇ₑᵣᵣ =
    1/2 qₑᵣᵣ ⊗ [0; ωₑᵣᵣ]

### Step 6: Virtual Control

    [0; ωₑᵣᵣ,𝒹] =
    -k₁ qₑᵣᵣ* ⊗
    (qₑᵣᵣ - qᵢ)

### Step 7: Backstepping Error

    δ = ωₑᵣᵣ - ωₑᵣᵣ,𝒹

### Step 8: Control Torque

    τ =
    ω × (Jω)
    + Jω̇ᵣₑ𝒻
    + Jω̇ₑᵣᵣ,𝒹
    - k₂Jδ¹ᐟ³

### Step 9: Final Error Dynamics

    δ̇ = -k₂δ¹ᐟ³

Therefore, the controller is designed so that:

    qₑᵣᵣ → qᵢ

    ωₑᵣᵣ → 0

    δ → 0

---

## 32. Project Workflow in Simple Terms

The controller works in the following way.

**First**, the actual quaternion is compared with the reference quaternion.

This gives the quaternion error:

    qₑᵣᵣ = qᵣₑ𝒻* ⊗ q

**Second**, the angular velocity error is calculated:

    ωₑᵣᵣ = ω - ωᵣₑ𝒻

**Third**, the controller calculates a desired angular velocity error using the quaternion error.

**Fourth**, the difference between the actual and desired angular velocity errors is calculated:

    δ = ωₑᵣᵣ - ωₑᵣᵣ,𝒹

**Finally**, the controller calculates the required control torque.

The torque compensates for the nonlinear rotational dynamics and adds a feedback term to reduce the tracking error.

As the simulation progresses, the errors should approach zero.

---

## 33. Advantages of the Approach

### Quaternion Representation

- Avoids Euler-angle singularities.
- Suitable for 3D attitude representation.
- Provides a compact representation of orientation.

### Backstepping Control

- Handles the nonlinear structure of the system.
- Provides a systematic controller design procedure.
- Separates the design into virtual-control and torque-control stages.

### Fractional-Power Feedback

The term:

    δ¹ᐟ³

is used in the final controller to drive the backstepping error toward zero.

---

## 34. Limitations

The current work is primarily based on numerical simulation.

The basic simulation does not necessarily include:

- Real quadrotor hardware.
- Real motor drivers.
- Real IMU sensors.
- Real-time flight testing.
- Wind and external disturbance testing.
- Hardware-in-the-loop testing.
- Actuator limitations.
- Sensor noise.

The research paper presents simulation results and identifies experimental validation as future work.

---

## 35. Future Scope

Possible future improvements include:

1. Implement the controller on a real quadrotor.
2. Integrate IMU sensor measurements.
3. Test the controller under wind disturbances.
4. Add actuator and motor dynamics.
5. Compare the backstepping controller with PID, LQR, and other nonlinear controllers.
6. Perform hardware-in-the-loop testing.
7. Extend attitude control to full position and trajectory control.
8. Analyze robustness under parameter uncertainty.

---

## 36. Research Paper

The project is based on:

**Quaternion-Based Attitude Tracking Control Design for UAVs**

Authors:

**Qain-Rong Lin and Jen-te Yu**

Conference:

**2024 International Automatic Control Conference (CACS 2024)**

Location:

**National Taiwan University of Science and Technology, Taipei**

Date:

**November 1–3, 2024**

DOI:

**10.1109/CACS63404.2024.10773309**

---

## 37. Conclusion

This project develops and simulates a quaternion-based backstepping attitude tracking controller for a quadrotor UAV.

The quadrotor is modeled using quaternion kinematics and rotational dynamics. The actual and desired attitudes are compared using quaternion error.

The controller first generates a virtual desired angular velocity error and then calculates the control torque required to make the actual angular velocity follow the desired behavior.

The final control torque is:

    τ =
    ω × (Jω)
    + Jω̇ᵣₑ𝒻
    + Jω̇ₑᵣᵣ,𝒹
    - k₂Jδ¹ᐟ³

with:

    δ = ωₑᵣᵣ - ωₑᵣᵣ,𝒹

and:

    ωₑᵣᵣ = ω - ωᵣₑ𝒻

The simulation evaluates quaternion error, attitude tracking, angular velocity tracking, backstepping error δ, and control torque.

The results reported in the research paper show that the tracking errors converge toward their desired values.

---

acking Application," IEEE Access, vol. 8, pp. 5515–5525, 2019.
