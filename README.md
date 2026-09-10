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

    q = [q<sub>0</sub>, q<sub>1</sub>, q<sub>2</sub>, q<sub>3</sub>]<sup>T</sup>

It can also be written as:

    q = q<sub>0</sub> + i q<sub>1</sub> + j q<sub>2</sub> + k q<sub>3</sub>

where:

- q<sub>0</sub> is the scalar component.
- q<sub>1</sub>, q<sub>2</sub>, q<sub>3</sub> are the vector components.

For a unit quaternion:

    ||q|| = √(q<sub>0</sub><sup>2</sup> + q<sub>1</sub><sup>2</sup> + q<sub>2</sub><sup>2</sup> + q<sub>3</sub><sup>2</sup>) = 1

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
    │ q<sub>err</sub> = q<sub>ref</sub>* ⊗ q             │
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

    q = [q<sub>0</sub>, q<sub>1</sub>, q<sub>2</sub>, q<sub>3</sub>]<sup>T</sup>

where:

- q<sub>0</sub> is the scalar component.
- q<sub>1</sub>, q<sub>2</sub>, q<sub>3</sub> are the vector components.

For a valid unit quaternion:

    ||q|| = 1

---

### 7.2 Quaternion Conjugate

The quaternion conjugate is:

    q* = [q<sub>0</sub>, -q<sub>1</sub>, -q<sub>2</sub>, -q<sub>3</sub>]<sup>T</sup>

The general quaternion inverse is:

    q<sup>-1</sup> = q* / ||q||<sup>2</sup>

For a unit quaternion:

    q<sup>-1</sup> = q*

The conjugate is used when calculating the attitude error.

---

### 7.3 Quaternion Normalization

A quaternion can be normalized using:

    q<sub>normalized</sub> = q / ||q||

Normalization is useful in numerical simulation to keep the quaternion close to unit length.

---

### 7.4 Quaternion Multiplication

Quaternion multiplication is written as:

    q<sub>1</sub> ⊗ q<sub>2</sub>

For:

    q<sub>1</sub> = [a, b, c, d]<sup>T</sup>
    q<sub>2</sub> = [e, f, g, h]<sup>T</sup>

the product is:

    q<sub>1</sub> ⊗ q<sub>2</sub> =

    [
    ae - bf - cg - dh
    af + be + ch - dg
    ag - bh + ce + df
    ah + bg - cf + de
    ]

Quaternion multiplication is non-commutative:

    q<sub>1</sub> ⊗ q<sub>2</sub> ≠ q<sub>2</sub> ⊗ q<sub>1</sub>

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

    ω̇ = J<sup>-1</sup> [τ - ω × (Jω)]

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

    q<sub>err</sub> = q<sub>ref</sub><sup>*</sup> ⊗ q

where:

- q<sub>ref</sub> = desired quaternion
- q = actual quaternion
- q<sub>ref</sub><sup>*</sup> = conjugate of the desired quaternion
- ⊗ = quaternion multiplication

When the actual and desired attitudes are equal:

    q = q<sub>ref</sub>

then:

    q<sub>err</sub> = [1, 0, 0, 0]<sup>T</sup>

This represents zero attitude error.

The identity quaternion is:

    q<sub>I</sub> = [1, 0, 0, 0]<sup>T</sup>

---

### 9.2 Angular Velocity Error

The angular velocity tracking error is:

    ω<sub>err</sub> = ω - ω<sub>ref</sub>

where:

- ω = actual angular velocity
- ω<sub>ref</sub> = reference angular velocity

The control objective is:

    ω<sub>err</sub> → 0

---

## 10. Quaternion Error Dynamics

The quaternion error dynamics are:

    q̇<sub>err</sub> = 1/2 q<sub>err</sub> ⊗ [0; ω<sub>err</sub>]

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

    δ = ω<sub>err</sub> - ω<sub>err,d</sub>

where:

- ω<sub>err</sub> = angular velocity error
- ω<sub>err,d</sub> = desired angular velocity error
- δ = backstepping error

---

## 12. Stage 1: Virtual Control

The virtual control is defined as:

    [0; ω<sub>err,d</sub>] =
    -k<sub>1</sub> q<sub>err</sub><sup>*</sup> ⊗
    (q<sub>err</sub> - q<sub>I</sub>)

where:

- k<sub>1</sub> > 0
- q<sub>I</sub> = [1, 0, 0, 0]<sup>T</sup>

This virtual control is designed to make the quaternion error converge toward the identity quaternion.

---

## 13. Stable Quaternion Error Dynamics

After substituting the virtual control into the quaternion error dynamics, the desired closed-loop behavior becomes:

    q̇<sub>err</sub> =
    -(k<sub>1</sub>/2)(q<sub>err</sub> - q<sub>I</sub>)

This means that the quaternion error is driven toward:

    q<sub>err</sub> = q<sub>I</sub>

or:

    q<sub>err</sub> = [1, 0, 0, 0]<sup>T</sup>

---

## 14. Stage 2: Backstepping Error

The second error variable is:

    δ = ω<sub>err</sub> - ω<sub>err,d</sub>

where:

    ω<sub>err</sub> = ω - ω<sub>ref</sub>

and:

    ω<sub>err,d</sub> = desired angular velocity error

The control torque is designed to make:

    δ → 0

---

## 15. Error Dynamics

Starting from:

    δ = ω<sub>err</sub> - ω<sub>err,d</sub>

differentiate both sides:

    δ̇ = ω̇<sub>err</sub> - ω̇<sub>err,d</sub>

Since:

    ω<sub>err</sub> = ω - ω<sub>ref</sub>

we have:

    ω̇<sub>err</sub> = ω̇ - ω̇<sub>ref</sub>

Using the quadrotor rotational dynamics:

    ω̇ = J<sup>-1</sup> [τ - ω × (Jω)]

therefore:

    δ̇ =
    J<sup>-1</sup> [τ - ω × (Jω)]
    - ω̇<sub>ref</sub>
    - ω̇<sub>err,d</sub>

This equation is used to design the final control torque.

---

## 16. Control Torque Design

The control torque is divided into two terms:

    τ = τ<sub>1</sub> + τ<sub>2</sub>

where:

- τ<sub>1</sub> compensates for the system dynamics and reference motion.
- τ<sub>2</sub> provides error convergence.

---

## 17. Torque Term 1: Dynamics Compensation

The first torque term is:

    τ<sub>1</sub> =
    ω × (Jω)
    + Jω̇<sub>ref</sub>
    + Jω̇<sub>err,d</sub>

This term compensates for:

- The nonlinear rotational dynamics.
- The reference angular velocity derivative.
- The derivative of the virtual control.

The derivative of the virtual control is obtained from:

    [0; ω<sub>err,d</sub>] =
    -k<sub>1</sub> q<sub>err</sub><sup>*</sup> ⊗
    (q<sub>err</sub> - q<sub>I</sub>)

Therefore:

    [0; ω̇<sub>err,d</sub>] =
    d/dt {
    -k<sub>1</sub> q<sub>err</sub><sup>*</sup> ⊗
    (q<sub>err</sub> - q<sub>I</sub>)
    }

---

## 18. Torque Term 2: Error Convergence

The second torque term is:

    τ<sub>2</sub> = -k<sub>2</sub> Jδ<sup>1/3</sup>

where:

    k<sub>2</sub> > 0

This fractional-power feedback term is used to drive the backstepping error toward zero.

The resulting error dynamics become:

    δ̇ = -k<sub>2</sub>δ<sup>1/3</sup>

---

## 19. Final Control Torque

Combining the two torque terms:

    τ = τ<sub>1</sub> + τ<sub>2</sub>

Therefore:

    τ =
    ω × (Jω)
    + Jω̇<sub>ref</sub>
    + Jω̇<sub>err,d</sub>
    - k<sub>2</sub>Jδ<sup>1/3</sup>

This is the main control law used in the simulation.

---

## 20. Final Error Dynamics

After substituting the control torque into the error dynamics:

    δ̇ = -k<sub>2</sub>δ<sup>1/3</sup>

The controller therefore drives:

    δ → 0

As the backstepping error converges, the angular velocity tracking error also converges toward the desired behavior.

---

## 21. Reference Trajectory

The reference angular velocity consists of three sinusoidal signals.

The paper describes the reference trajectory as being arbitrarily defined using sine waves.

The reference signal is represented as:

    ω<sub>ref</sub> =
    [ω<sub>ref,1</sub>, ω<sub>ref,2</sub>, ω<sub>ref,3</sub>]<sup>T</sup>

Its derivative is:

    ω̇<sub>ref</sub> = d(ω<sub>ref</sub>)/dt

The exact amplitudes and frequencies should be taken from the MATLAB implementation when running the simulation.

---

## 22. Simulation Parameters

The parameters used in the paper are:

| Parameter | Value |
|---|---|
| J | diag(0.1, 0.1, 0.12) |
| k<sub>1</sub> | 20 |
| k<sub>2</sub> | 2 |

Initial angular velocity:

    ω(0) = [-0.1, -0.2, 0.2]<sup>T</sup>

Initial reference angular velocity:

    ω<sub>ref</sub>(0) = [-0.1, -0.2, 0.2]<sup>T</sup>

Initial quaternion:

    q(0) = [0.94628, -0.1541, 0.19051, -0.21098]<sup>T</sup>

Initial reference quaternion:

    q<sub>ref</sub>(0) =
    [0.94628, -0.1541, 0.19051, -0.21098]<sup>T</sup>

---

## 23. Closed-Loop Simulation

The complete system is simulated as a closed-loop system.

The reference trajectory is given to the controller.

The controller calculates:

    q<sub>err</sub>

then:

    ω<sub>err</sub>

then:

    ω<sub>err,d</sub>

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
    4. Calculate q<sub>ref</sub>*.
    5. Calculate q<sub>err</sub> = q<sub>ref</sub>* ⊗ q.
    6. Calculate ω<sub>err</sub> = ω - ω<sub>ref</sub>.
    7. Calculate the virtual control ω<sub>err,d</sub>.
    8. Calculate δ = ω<sub>err</sub> - ω<sub>err,d</sub>.
    9. Calculate ω̇<sub>err,d</sub>.
    10. Calculate τ<sub>1</sub>.
    11. Calculate τ<sub>2</sub>.
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

    q<sub>err</sub> → [1, 0, 0, 0]<sup>T</sup>

### Angular Velocity Error

The angular velocity error should converge toward:

    ω<sub>err</sub> → [0, 0, 0]<sup>T</sup>

### Backstepping Error

The error:

    δ = ω<sub>err</sub> - ω<sub>err,d</sub>

should converge toward:

    δ → [0, 0, 0]<sup>T</sup>

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

    q<sub>err</sub>
    ω<sub>err</sub>
    ω<sub>err,d</sub>
    δ
    τ<sub>1</sub>
    τ<sub>2</sub>
    τ

The dynamics function calculates:

    q_dot
    ω_dot

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
| q<sub>ref</sub> | Reference quaternion |
| q<sub>err</sub> | Quaternion attitude error |
| q<sub>I</sub> | Identity quaternion |
| ω | Actual angular velocity |
| ω<sub>ref</sub> | Reference angular velocity |
| ω<sub>err</sub> | Angular velocity error |
| ω<sub>err,d</sub> | Desired angular velocity error |
| δ | Backstepping error |
| J | Inertia matrix |
| τ | Total control torque |
| τ<sub>1</sub> | Dynamics compensation torque |
| τ<sub>2</sub> | Error convergence torque |
| k<sub>1</sub> | First controller gain |
| k<sub>2</sub> | Second controller gain |

---

## 31. Complete Mathematical Flow

The complete controller can be summarized as follows.

### Step 1: Quaternion Kinematics

    q̇ = 1/2 q ⊗ [0; ω]

### Step 2: Rotational Dynamics

    ω̇ = J<sup>-1</sup> [τ - ω × (Jω)]

### Step 3: Quaternion Error

    q<sub>err</sub> = q<sub>ref</sub><sup>*</sup> ⊗ q

### Step 4: Angular Velocity Error

    ω<sub>err</sub> = ω - ω<sub>ref</sub>

### Step 5: Quaternion Error Dynamics

    q̇<sub>err</sub> =
    1/2 q<sub>err</sub> ⊗ [0; ω<sub>err</sub>]

### Step 6: Virtual Control

    [0; ω<sub>err,d</sub>] =
    -k<sub>1</sub> q<sub>err</sub><sup>*</sup> ⊗
    (q<sub>err</sub> - q<sub>I</sub>)

### Step 7: Backstepping Error

    δ = ω<sub>err</sub> - ω<sub>err,d</sub>

### Step 8: Control Torque

    τ =
    ω × (Jω)
    + Jω̇<sub>ref</sub>
    + Jω̇<sub>err,d</sub>
    - k<sub>2</sub>Jδ<sup>1/3</sup>

### Step 9: Final Error Dynamics

    δ̇ = -k<sub>2</sub>δ<sup>1/3</sup>

Therefore, the controller is designed so that:

    q<sub>err</sub> → q<sub>I</sub>

    ω<sub>err</sub> → 0

    δ → 0

---

## 32. Project Workflow in Simple Terms

The controller works in the following way.

**First**, the actual quaternion is compared with the reference quaternion.

This gives the quaternion error:

    q<sub>err</sub> = q<sub>ref</sub><sup>*</sup> ⊗ q

**Second**, the angular velocity error is calculated:

    ω<sub>err</sub> = ω - ω<sub>ref</sub>

**Third**, the controller calculates a desired angular velocity error using the quaternion error.

**Fourth**, the difference between the actual and desired angular velocity errors is calculated:

    δ = ω<sub>err</sub> - ω<sub>err,d</sub>

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

    δ<sup>1/3</sup>

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
    + Jω̇<sub>ref</sub>
    + Jω̇<sub>err,d</sub>
    - k<sub>2</sub>Jδ<sup>1/3</sup>

with:

    δ = ω<sub>err</sub> - ω<sub>err,d</sub>

and:

    ω<sub>err</sub> = ω - ω<sub>ref</sub>

The simulation evaluates quaternion error, attitude tracking, angular velocity tracking, backstepping error δ, and control torque.

The results reported in the research paper show that the tracking errors converge toward their desired values.

---

