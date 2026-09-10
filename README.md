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

This project focuses on attitude tracking control of a quadrotor UAV using quaternions.

The main idea is to control the orientation of the quadrotor so that its actual attitude follows a desired attitude trajectory. The controller is designed using a backstepping approach with a fractional-power feedback term.

The mathematical model and controller are based on the research paper:

**"Quaternion-Based Attitude Tracking Control Design for UAVs"**

The controller is tested through numerical simulation using the parameters given in the paper.

---

## 2. Problem Statement

A quadrotor must continuously adjust its orientation while flying.

The attitude of the UAV is described by its orientation and angular velocity. The control objective is to make the actual attitude and angular velocity follow their desired reference values.

The main challenges are:

- Attitude is nonlinear.
- Rotational dynamics are coupled.
- Quaternion multiplication is non-commutative.
- Euler angles can suffer from singularities.
- The controller must compensate for the nonlinear rotational dynamics.

To handle these issues, this project uses quaternion-based attitude representation and a backstepping controller.

---

## 3. Project Objectives

The main objectives are:

1. Represent quadrotor attitude using quaternions.
2. Develop the quaternion-based attitude error.
3. Define angular velocity tracking error.
4. Design a virtual control using backstepping.
5. Design the final control torque.
6. Simulate the closed-loop system.
7. Check whether the attitude and angular velocity errors converge to zero.
8. Compare the simulation behavior with the results reported in the research paper.

---

## 4. Why Quaternions?

Quaternions provide a convenient way to represent 3D rotations.

A quaternion is written as:

    q = q0 + i*q1 + j*q2 + k*q3

In vector form:

    q = [q0, q1, q2, q3]^T

For a unit quaternion:

    ||q|| = sqrt(q0^2 + q1^2 + q2^2 + q3^2) = 1

Here:

- q0 is the scalar component.
- q1, q2, q3 are the vector components.

Quaternions are useful for UAV attitude control because they avoid the singularity problems associated with Euler angles.

---

## 5. Quaternion Operations

### 5.1 Quaternion Conjugate

For

    q = [q0, q1, q2, q3]^T

the conjugate is:

    q* = [q0, -q1, -q2, -q3]^T

---

### 5.2 Quaternion Norm

The quaternion norm is:

    ||q|| = sqrt(q0^2 + q1^2 + q2^2 + q3^2)

For attitude representation, the quaternion is normalized so that:

    ||q|| = 1

---

### 5.3 Quaternion Inverse

The general quaternion inverse is:

    q^-1 = q* / ||q||^2

For a unit quaternion:

    q^-1 = q*

---

### 5.4 Quaternion Normalization

A quaternion can be normalized using:

    q_normalized = q / ||q||

This is useful in numerical simulation to reduce numerical drift.

---

### 5.5 Quaternion Multiplication

Quaternion multiplication is written as:

    q = q1 ⊗ q2

For:

    q1 = [a, b, c, d]^T
    q2 = [e, f, g, h]^T

the product is:

    q1 ⊗ q2 =
    [
        ae - bf - cg - dh
        af + be + ch - dg
        ag - bh + ce + df
        ah + bg - cf + de
    ]

Quaternion multiplication is non-commutative:

    q1 ⊗ q2 != q2 ⊗ q1

Therefore, the order of quaternion multiplication must be maintained correctly.

---

## 6. Quadrotor Attitude Dynamics

The quadrotor rotational dynamics are given by:

    ω_dot = J^-1 [ τ - ω × (Jω) ]

where:

- ω = angular velocity
- J = inertia matrix
- τ = control torque
- × = cross product

The quaternion kinematics are:

    q_dot = 1/2 * q ⊗ [0; ω]

where:

    [0; ω] = [0, ω1, ω2, ω3]^T

---

## 7. Attitude Error

The quaternion attitude error is defined as:

    q_err = q_ref* ⊗ q

where:

- q_ref = desired/reference quaternion
- q = actual quaternion
- q_ref* = conjugate of the reference quaternion

When the actual attitude matches the reference attitude:

    q_err = [1, 0, 0, 0]^T

The identity quaternion is defined as:

    q_I = [1, 0, 0, 0]^T

Using q_I instead of I avoids confusion with the identity matrix used elsewhere.

---

## 8. Angular Velocity Error

The angular velocity tracking error is:

    ω_err = ω - ω_ref

where:

- ω = actual angular velocity
- ω_ref = reference angular velocity

The objective of the controller is:

    ω_err -> 0

---

## 9. Quaternion Error Dynamics

The error quaternion dynamics are:

    q_err_dot = 1/2 * q_err ⊗ [0; ω_err]

This equation connects the attitude error with the angular velocity tracking error.

The controller first creates a desired angular velocity error, which is then used in the second backstepping stage to generate the control torque.

---

## 10. Backstepping Controller

The controller is designed in two stages.

### Stage 1

Design a virtual control for the angular velocity error.

### Stage 2

Use the difference between the actual angular velocity error and the virtual control to generate the final control torque.

The two main errors are:

    q_err

and

    δ = ω_err - ω_err,d

where:

- ω_err,d = desired angular velocity error
- δ = backstepping error

---

## 11. Stage 1: Virtual Control

The virtual control is defined as:

    [0; ω_err,d] =
    -k1 * q_err* ⊗ (q_err - q_I)

where:

- k1 > 0
- q_I = [1, 0, 0, 0]^T

This virtual control is designed to make the quaternion error converge toward the identity quaternion.

---

## 12. Stable Quaternion Error Dynamics

After substituting the virtual control into the quaternion error dynamics, the desired closed-loop behavior becomes:

    q_err_dot = -(k1/2) * (q_err - q_I)

This means that the quaternion error is driven toward:

    q_err = q_I

or:

    q_err = [1, 0, 0, 0]^T

---

## 13. Stage 2: Backstepping Error

The second error variable is defined as:

    δ = ω_err - ω_err,d

where:

    ω_err = ω - ω_ref

and:

    ω_err,d = desired angular velocity error

The control torque is designed to make:

    δ -> 0

---

## 14. Error Dynamics

Starting from:

    δ = ω_err - ω_err,d

differentiate both sides:

    δ_dot = ω_err_dot - ω_err,d_dot

Since:

    ω_err = ω - ω_ref

we have:

    ω_err_dot = ω_dot - ω_ref_dot

Using the quadrotor rotational dynamics:

    ω_dot = J^-1 [ τ - ω × (Jω) ]

therefore:

    δ_dot =
    J^-1 [ τ - ω × (Jω) ]
    - ω_ref_dot
    - ω_err,d_dot

This equation is used to design the final control torque.

---

## 15. Control Torque Design

The control torque is divided into two terms:

    τ = τ1 + τ2

where:

- τ1 compensates for the system dynamics and reference motion.
- τ2 provides error convergence.

---

## 16. Torque Term 1: Dynamics Compensation

The first torque term is:

    τ1 =
    ω × (Jω)
    + J * ω_ref_dot
    + J * ω_err,d_dot

This term compensates for:

- The nonlinear rotational dynamics.
- The reference angular velocity derivative.
- The derivative of the virtual control.

The derivative of the virtual control is obtained from:

    [0; ω_err,d] =
    -k1 * q_err* ⊗ (q_err - q_I)

Therefore:

    [0; ω_err,d_dot]
    =
    d/dt { -k1 * q_err* ⊗ (q_err - q_I) }

---

## 17. Torque Term 2: Error Convergence

The second torque term is:

    τ2 = -k2 * J * δ^(1/3)

where:

    k2 > 0

This fractional-power feedback term is used to drive the backstepping error toward zero.

The resulting error dynamics become:

    δ_dot = -k2 * δ^(1/3)

---

## 18. Final Control Torque

Combining the two torque terms:

    τ = τ1 + τ2

Therefore:

    τ =
    ω × (Jω)
    + J * ω_ref_dot
    + J * ω_err,d_dot
    - k2 * J * δ^(1/3)

This is the main control law used in the simulation.

---

## 19. Final Error Dynamics

After substituting the control torque into the error dynamics:

    δ_dot = -k2 * δ^(1/3)

The controller therefore drives:

    δ -> 0

As the backstepping error converges, the angular velocity tracking error also converges toward the desired behavior.

---

## 20. Reference Trajectory

The reference angular velocity is generated using three sinusoidal signals.

The paper describes the reference trajectory as being arbitrarily defined using sine waves.

The exact amplitudes and frequencies are not specified in the main equations reproduced here, so they should be taken directly from the MATLAB implementation when running the simulation.

The reference signal is represented as:

    ω_ref = [ω_ref1, ω_ref2, ω_ref3]^T

The derivative is:

    ω_ref_dot = d(ω_ref)/dt

---

## 21. Simulation Parameters

The parameters used in the paper are:

| Parameter | Value |
|---|---|
| J | diag(0.1, 0.1, 0.12) |
| k1 | 20 |
| k2 | 2 |

Initial angular velocity:

    ω(0) = [-0.1, -0.2, 0.2]^T

Initial reference angular velocity:

    ω_ref(0) = [-0.1, -0.2, 0.2]^T

Initial quaternion:

    q(0) =
    [0.94628, -0.1541, 0.19051, -0.21098]^T

Initial reference quaternion:

    q_ref(0) =
    [0.94628, -0.1541, 0.19051, -0.21098]^T

---

## 22. Closed-Loop Simulation

The complete system is simulated as a closed-loop system.

The reference trajectory is given to the controller.

The controller calculates:

    q_err

then:

    ω_err

then:

    ω_err,d

then:

    δ

and finally:

    τ

The calculated torque is applied to the quadrotor rotational dynamics.

The updated angular velocity is then used to update the quaternion.

This process continues at every simulation step.

---

## 23. Simulation Procedure

The basic simulation loop is:

    1. Generate the reference trajectory.
    2. Read the current quaternion q.
    3. Read the current angular velocity ω.
    4. Calculate q_ref*.
    5. Calculate q_err = q_ref* ⊗ q.
    6. Calculate ω_err = ω - ω_ref.
    7. Calculate the virtual control ω_err,d.
    8. Calculate δ = ω_err - ω_err,d.
    9. Calculate ω_err,d_dot.
    10. Calculate τ1.
    11. Calculate τ2.
    12. Calculate the total torque τ.
    13. Update angular velocity using the rotational dynamics.
    14. Update quaternion using the quaternion kinematics.
    15. Normalize the quaternion.
    16. Store the results.
    17. Plot the tracking errors and control torque.

---

## 24. Expected Simulation Results

The controller should produce the following behavior:

### Quaternion Error

The quaternion error should converge toward:

    [1, 0, 0, 0]^T

### Angular Velocity Error

The angular velocity error should converge toward:

    [0, 0, 0]^T

### Backstepping Error

The error:

    δ = ω_err - ω_err,d

should converge toward:

    [0, 0, 0]^T

### Control Torque

The control torque should decrease as the system approaches the desired trajectory.

---

## 25. Results Reported in the Research Paper

The research paper reports that:

- The quaternion error converges toward [1, 0, 0, 0].
- The angular velocity error approaches zero at approximately 0.8 seconds.
- The backstepping error δ approaches zero at approximately 0.4 seconds.
- The control torque approaches zero when the system reaches equilibrium.

These results indicate that the proposed controller is able to track the desired attitude trajectory in simulation.

---

## 26. Software Requirements

The project can be implemented using MATLAB.

Recommended tools:

- MATLAB
- MATLAB scripts/functions
- Numerical ODE solver
- Plotting tools

No physical quadrotor hardware is required for the numerical simulation.

---

## 27. MATLAB Implementation

A MATLAB implementation can be organized around the following functions:

    quatMultiply
    quatConjugate
    quatNormalize
    quatInverse

These functions handle the basic quaternion operations.

The controller function can calculate:

    q_err
    ω_err
    ω_err,d
    δ
    τ1
    τ2
    τ

The dynamics function can calculate:

    q_dot
    ω_dot

---

## 28. Suggested Project Structure

A simple MATLAB project structure is:

    Drones_S5_AB11/
    |
    |-- README.md
    |
    |-- main.m
    |
    |-- controller.m
    |
    |-- dynamics.m
    |
    |-- referenceTrajectory.m
    |
    |-- quatMultiply.m
    |
    |-- quatConjugate.m
    |
    |-- quatNormalize.m
    |
    |-- quatInverse.m
    |
    |-- plots.m
    |
    |-- results/
    |
    |   |-- quaternion_error.png
    |   |-- angular_velocity_error.png
    |   |-- delta.png
    |   |-- control_torque.png

The exact file structure can be changed depending on the MATLAB implementation.

---

## 29. Main Program Flow

The main program follows this general sequence:

    Start
      |
      v
    Initialize parameters
      |
      v
    Initialize q and ω
      |
      v
    Generate reference trajectory
      |
      v
    Calculate quaternion error
      |
      v
    Calculate angular velocity error
      |
      v
    Calculate virtual control
      |
      v
    Calculate δ
      |
      v
    Calculate control torque
      |
      v
    Update quadrotor dynamics
      |
      v
    Update quaternion
      |
      v
    Normalize quaternion
      |
      v
    Store results
      |
      v
    Plot results
      |
      v
    End

---

## 30. Important Variables

| Variable | Meaning |
|---|---|
| q | Actual quaternion |
| q_ref | Reference quaternion |
| q_err | Quaternion attitude error |
| q_I | Identity quaternion |
| ω | Actual angular velocity |
| ω_ref | Reference angular velocity |
| ω_err | Angular velocity error |
| ω_err,d | Desired angular velocity error |
| δ | Backstepping error |
| J | Inertia matrix |
| τ | Total control torque |
| τ1 | Dynamics compensation torque |
| τ2 | Error convergence torque |
| k1 | First controller gain |
| k2 | Second controller gain |

---

## 31. Complete Mathematical Flow

The complete controller can be summarized as follows.

### Step 1: Quaternion Kinematics

    q_dot = 1/2 * q ⊗ [0; ω]

### Step 2: Rotational Dynamics

    ω_dot = J^-1 [ τ - ω × (Jω) ]

### Step 3: Quaternion Error

    q_err = q_ref* ⊗ q

### Step 4: Angular Velocity Error

    ω_err = ω - ω_ref

### Step 5: Quaternion Error Dynamics

    q_err_dot = 1/2 * q_err ⊗ [0; ω_err]

### Step 6: Virtual Control

    [0; ω_err,d] =
    -k1 * q_err* ⊗ (q_err - q_I)

### Step 7: Backstepping Error

    δ = ω_err - ω_err,d

### Step 8: Control Torque

    τ =
    ω × (Jω)
    + J * ω_ref_dot
    + J * ω_err,d_dot
    - k2 * J * δ^(1/3)

### Step 9: Final Error Dynamics

    δ_dot = -k2 * δ^(1/3)

Therefore, the controller is designed so that:

    q_err -> q_I

    ω_err -> 0

    δ -> 0

---

## 32. Project Workflow in Simple Terms

The controller works in the following way:

**First**, the actual quaternion is compared with the reference quaternion.

This gives the quaternion error:

    q_err = q_ref* ⊗ q

**Second**, the angular velocity error is calculated:

    ω_err = ω - ω_ref

**Third**, the controller calculates a desired angular velocity error using the quaternion error.

**Fourth**, the difference between the actual and desired angular velocity errors is calculated:

    δ = ω_err - ω_err,d

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

    δ^(1/3)

is used in the final controller to drive the backstepping error toward zero.

---

## 34. Limitations

The current work is based on numerical simulation.

Some practical effects are not covered in the basic simulation, such as:

- Sensor noise.
- Actuator saturation.
- External disturbances.
- Aerodynamic effects.
- Rotor dynamics.
- Model uncertainty.
- Communication delays.
- Hardware limitations.

The research paper also states that experimental validation is a topic for future work.

---

## 35. Future Scope

Possible extensions include:

- Testing the controller on a real quadrotor.
- Adding external disturbance rejection.
- Considering actuator saturation.
- Adding sensor noise to the simulation.
- Testing robustness against model uncertainty.
- Implementing the controller on embedded hardware.
- Comparing the controller with PID, LQR, sliding-mode, and other attitude controllers.
- Testing the controller under different reference trajectories.

---

## 36. Research Paper

The controller is based on:

**Quaternion-Based Attitude Tracking Control Design for UAVs**

Authors:

**Qain-Rong Lin and Jen-te Yu**

Conference:

**2024 International Automatic Control Conference (CACS 2024)**

Location:

**NTUST, Taipei**

Date:

**November 1–3, 2024**

DOI:

**10.1109/CACS63404.2024.10773309**

---

## 37. Conclusion

This project implements a quaternion-based attitude tracking controller for a quadrotor UAV.

The approach uses quaternion attitude representation, angular velocity tracking error, and a two-stage backstepping controller.

The first stage generates a virtual angular velocity error, while the second stage generates the control torque required to reduce the remaining tracking error.

The final controller is:

    τ =
    ω × (Jω)
    + J * ω_ref_dot
    + J * ω_err,d_dot
    - k2 * J * δ^(1/3)

with:

    δ = ω_err - ω_err,d

and:

    ω_err = ω - ω_ref

The simulation results reported in the research paper show that the quaternion error, angular velocity error, and backstepping error converge toward their desired values.

---

## 38. Key Takeaways

- Quaternions are used to represent quadrotor attitude.
- Quaternion multiplication must be performed in the correct order.
- The attitude error is:

      q_err = q_ref* ⊗ q

- The angular velocity error is:

      ω_err = ω - ω_ref

- A virtual control is generated using the quaternion error.
- The second backstepping stage generates the control torque.
- The final torque contains both dynamics compensation and error feedback.
- The fractional-power term δ^(1/3) is used for error convergence.
- Numerical simulation is used to evaluate the controller.
- The reported results show convergence of the tracking errors.

---

## References

1. Qain-Rong Lin and Jen-te Yu, "Quaternion-Based Attitude Tracking Control Design for UAVs," 2024 International Automatic Control Conference (CACS 2024), NTUST, Taipei, 2024. DOI: 10.1109/CACS63404.2024.10773309.

2. Fresk and Nikolakopoulos, "Full Quaternion Based Attitude Control for a Quadrotor," European Control Conference (ECC), 2013.

3. Esmail et al., "Attitude and Altitude Tracking Controller for Quadcopter Dynamical Systems," IEEE Access, 2022.

4. Chovancová et al., "Comparison of Various Quaternion-Based Control Methods for a Quadrotor," Robotics and Autonomous Systems, 2016.

5. Kimathi and Lantos, "PD Control and Unwinding Problem in Quaternion-Based Attitude Control," IEEE INES, 2023.

6. Reyes-Valeria et al., "LQR Control Using Unit Quaternions," 2013.

7. Jen-te Yu, "A Unified SO(3) Approach to the Attitude Control Design for Quadrotors," IEEE Access, 2021.

8. Meslouli et al., "Experimental Validation of Quaternion Based Integral Backstepping Design for Attitude Tracking," CEIT, 2018.

9. Lindqvist et al., "Nonlinear Model Predictive Control for Dynamic Obstacle Avoidance," IEEE Robotics and Automation Letters, 2020.

10. Almakhles, "Robust Backstepping Sliding Mode Control for Quadrotor UAVs," IEEE Access, 2019.
