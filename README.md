<p align="center">
  <img src="Amrita Vishwa Vidhyapeetam" width="200">
</p>



# Quaternion-Based Attitude Tracking Control for a Quadrotor UAV

## Team Members

| Name | Roll Number |
|---|---|
| N.Supreeth | CB.SC.U4AIE24139 |
| P.Rohit | CB.SC.U4AIE24145 |
| B.Nikhil | CB.SC.U4AIE24063 |
| M.Phanendhra | CB.SC.U4AIE24032 |
| D.Koushik | CB.SC.U4AIE24167 |

---

# Title

## Quaternion-Based Attitude Tracking Control for a Quadrotor UAV

---

# Abstract

Quadrotor UAVs require accurate attitude control to maintain and change their orientation during flight. This project investigates a quaternion-based attitude tracking controller using a backstepping control approach.

Quaternions are used to represent the orientation of the quadrotor because they provide a compact representation of three-dimensional rotations without the singularities associated with Euler angles. The controller uses the difference between the desired and actual quaternion to determine the attitude error and then generates a desired angular velocity through a virtual control law.

A second backstepping stage is used to calculate the required control torque while accounting for the nonlinear rotational dynamics of the quadrotor. A fractional-power feedback term is included to reduce the backstepping error.

The complete controller is implemented and evaluated through numerical simulation in MATLAB. The simulation examines quaternion error, angular velocity tracking error, backstepping error, and control torque.

---

# Introduction

A quadrotor UAV is an underactuated nonlinear system whose motion depends on the interaction between its position, orientation, angular velocity, and control inputs. Among these, attitude control is particularly important because the orientation of the vehicle determines the direction in which the generated thrust acts.

The attitude of a quadrotor can be represented using Euler angles, rotation matrices, or quaternions. Although Euler angles are intuitive, they can suffer from gimbal lock. Quaternions provide an alternative representation that is well suited for describing three-dimensional orientation and nonlinear attitude control.

In this project, the attitude is represented using a unit quaternion:

```math
q = [q_0,\ q_1,\ q_2,\ q_3]^T
```

with the unit-quaternion constraint:

```math
\lVert q\rVert =
\sqrt{q_0^2 + q_1^2 + q_2^2 + q_3^2}
= 1
```

The main objective is to make the actual quadrotor attitude follow a desired attitude trajectory while ensuring that the angular velocity tracking error is reduced.

The control strategy is based on backstepping. Since torque directly affects angular velocity and angular velocity determines the evolution of the attitude, the controller is developed in stages. The first stage generates a desired angular velocity from the quaternion attitude error. The second stage determines the control torque required to make the actual angular velocity follow this desired behaviour.

This project is based on the research paper **"Quaternion-Based Attitude Tracking Control Design for UAVs"** by Qain-Rong Lin and Jen-te Yu, presented at the 2024 International Automatic Control Conference (CACS 2024).

---

# Methodology

## 1. Quaternion Representation

The quadrotor attitude is represented by a unit quaternion:

```math
q = [q_0,\ q_1,\ q_2,\ q_3]^T
```

The quaternion conjugate is given by:

```math
q^* =
[q_0,\ -q_1,\ -q_2,\ -q_3]^T
```

For a unit quaternion, the inverse is equal to its conjugate:

```math
q^{-1} = q^*
```

During numerical simulation, quaternion normalization is applied to maintain the unit-quaternion constraint:

```math
q_{\mathrm{norm}} =
\frac{q}{\lVert q\rVert}
```

---

## 2. Quaternion Multiplication

The multiplication of two quaternions is written as:

```math
q_1 \otimes q_2
```

For:

```math
q_1 = [a,\ b,\ c,\ d]^T
```

and:

```math
q_2 = [e,\ f,\ g,\ h]^T
```

their product is:

```math
q_1 \otimes q_2 =
\begin{bmatrix}
ae-bf-cg-dh \\
af+be+ch-dg \\
ag-bh+ce+df \\
ah+bg-cf+de
\end{bmatrix}
```

Quaternion multiplication is non-commutative:

```math
q_1 \otimes q_2 \neq q_2 \otimes q_1
```

Therefore, the order of multiplication is important when calculating the attitude error.

---





## Equation (8): Quaternion Kinematic Equation

### Equation

The quaternion kinematic equation is:

$$
\boxed{
\dot{q}=\frac{1}{2}q\otimes
\begin{bmatrix}
0\\
\boldsymbol{\omega}
\end{bmatrix}
}
$$

where:

- $q=[q_0,q_1,q_2,q_3]^T$ is the current attitude quaternion.
- $\dot{q}$ is the rate of change of the attitude quaternion.
- $\boldsymbol{\omega}=[\omega_x,\omega_y,\omega_z]^T$ is the angular velocity.
- $\otimes$ represents quaternion multiplication.

### Derivation of Equation (8)

**Step 1: Start with the quaternion representation of rotation.**

A rotation through an angle $\theta$ about a unit axis $\mathbf{u}$ is represented by:

$$
q=
\begin{bmatrix}
\cos(\theta/2)\\
\mathbf{u}\sin(\theta/2)
\end{bmatrix}
$$

The angle is divided by two because quaternions represent a three-dimensional rotation using the half-angle.

**Step 2: Relate angular velocity to the change in angle.**

Angular velocity describes how quickly the rotation angle changes:

$$
\boldsymbol{\omega}=\dot{\theta}\mathbf{u}
$$

Over a small time interval $\Delta t$, the small rotation is approximately:

$$
\Delta\boldsymbol{\theta}
\approx\boldsymbol{\omega}\Delta t
$$

The quaternion representing this small rotation is:

$$
\Delta q=
\begin{bmatrix}
\cos(\Delta\theta/2)\\
\mathbf{u}\sin(\Delta\theta/2)
\end{bmatrix}
$$

For a small angle, we use the approximations:

$$
\cos x\approx1,\qquad \sin x\approx x
$$

Therefore:

$$
\Delta q\approx
\begin{bmatrix}
1\\
\frac{1}{2}\boldsymbol{\omega}\Delta t
\end{bmatrix}
$$

**Step 3: Update the current quaternion.**

Using the multiplication convention in the research paper, the updated quaternion is:

$$
q(t+\Delta t)\approx q(t)\otimes\Delta q
$$

Substituting the small-rotation quaternion:

$$
q(t+\Delta t)\approx
q(t)\otimes
\begin{bmatrix}
1\\
\frac{1}{2}\boldsymbol{\omega}\Delta t
\end{bmatrix}
$$

Using the identity quaternion $[1,0,0,0]^T$ and the linearity of quaternion multiplication, we obtain:

$$
q(t+\Delta t)\approx
q(t)+\frac{\Delta t}{2}
q(t)\otimes
\begin{bmatrix}
0\\
\boldsymbol{\omega}
\end{bmatrix}
$$

**Step 4: Calculate the derivative.**

Subtract $q(t)$ from both sides:

$$
q(t+\Delta t)-q(t)\approx
\frac{\Delta t}{2}
q(t)\otimes
\begin{bmatrix}
0\\
\boldsymbol{\omega}
\end{bmatrix}
$$

Divide both sides by $\Delta t$:

$$
\frac{q(t+\Delta t)-q(t)}{\Delta t}
\approx
\frac{1}{2}
q(t)\otimes
\begin{bmatrix}
0\\
\boldsymbol{\omega}
\end{bmatrix}
$$

As $\Delta t\rightarrow0$, the left-hand side becomes the time derivative of the quaternion:

$$
\dot q=\lim_{\Delta t\rightarrow0}
\frac{q(t+\Delta t)-q(t)}{\Delta t}
$$

Therefore, we obtain Equation (8):

$$
\boxed{
\dot{q}=\frac{1}{2}q\otimes
\begin{bmatrix}
0\\
\boldsymbol{\omega}
\end{bmatrix}
}
$$

The factor $\frac{1}{2}$ comes from the half-angle used in the quaternion representation.

### What Does Equation (8) Do?

Equation (8) calculates how the drone's orientation changes with time based on its current attitude quaternion and angular velocity.

The quaternion $q$ represents the current orientation, while $\boldsymbol{\omega}$ describes the rotational motion. Combining them gives $\dot q$, which tells us how quickly the orientation quaternion is changing.

In the simulation, this equation is integrated over time to continuously update the drone's orientation as it rotates.

**Why Do We Use This Equation in Our Project?**

Our controller must make the actual drone orientation follow a desired orientation. Equation (8) provides the relationship needed to update the actual orientation and evaluate its change during the simulation.

In summary:

- **Quaternion:** Represents the drone's current orientation.
- **Angular velocity:** Describes how quickly the drone rotates.
- **Quaternion derivative:** Describes how quickly the orientation changes.

Equation (8) is the **kinematic model** of the drone's attitude. The next equation, Equation (9), describes the rotational dynamics connecting applied torque to angular acceleration.






## Equation (9): Rotational Dynamics of the Quadrotor

### Equation

The rotational dynamics of the quadrotor are described by:

```math
\boxed{
\dot{\boldsymbol{\omega}}
=
J^{-1}
\left[
\boldsymbol{\tau}
-
\boldsymbol{\omega}\times(J\boldsymbol{\omega})
\right]
}
```

### Meaning of the Variables

- Moment of inertia matrix:

```math
J
```

- Angular velocity vector:

```math
\boldsymbol{\omega}
=
\begin{bmatrix}
\omega_x\\
\omega_y\\
\omega_z
\end{bmatrix}
```

- Angular acceleration vector:

```math
\dot{\boldsymbol{\omega}}
=
\begin{bmatrix}
\dot{\omega}_x\\
\dot{\omega}_y\\
\dot{\omega}_z
\end{bmatrix}
```

- Applied control torque:

```math
\boldsymbol{\tau}
=
\begin{bmatrix}
\tau_x\\
\tau_y\\
\tau_z
\end{bmatrix}
```

- Angular momentum in body coordinates:

```math
\mathbf{H}=J\boldsymbol{\omega}
```

- Gyroscopic coupling term:

```math
\boldsymbol{\omega}\times(J\boldsymbol{\omega})
```

### Derivation of Equation (9)

**Step 1: Start with Euler's equation of rotational motion.**

The applied torque equals the rate of change of angular momentum measured in the inertial reference frame.

```math
\boldsymbol{\tau}
=
\left(\frac{d\mathbf{H}}{dt}\right)_{\mathrm{inertial}}
```

**Step 2: Express angular momentum using angular velocity.**

For a rigid body:

```math
\mathbf{H}=J\boldsymbol{\omega}
```

Substituting into Euler's equation gives:

```math
\boldsymbol{\tau}
=
\left(
\frac{d(J\boldsymbol{\omega})}{dt}
\right)_{\mathrm{inertial}}
```

**Step 3: Account for the rotating body coordinate system.**

The relationship between the inertial derivative and the derivative measured in body coordinates is:

```math
\left(\frac{d\mathbf{H}}{dt}\right)_{\mathrm{inertial}}
=
\left(\frac{d\mathbf{H}}{dt}\right)_{\mathrm{body}}
+
\boldsymbol{\omega}\times\mathbf{H}
```

Substituting angular momentum:

```math
\boldsymbol{\tau}
=
\left(
\frac{d(J\boldsymbol{\omega})}{dt}
\right)_{\mathrm{body}}
+
\boldsymbol{\omega}\times(J\boldsymbol{\omega})
```

**Step 4: Differentiate angular momentum.**

Assuming the inertia matrix is constant in body coordinates:

```math
\left(
\frac{d(J\boldsymbol{\omega})}{dt}
\right)_{\mathrm{body}}
=
J\dot{\boldsymbol{\omega}}
```

Substituting this result:

```math
\boldsymbol{\tau}
=
J\dot{\boldsymbol{\omega}}
+
\boldsymbol{\omega}\times(J\boldsymbol{\omega})
```

**Step 5: Rearrange to obtain angular acceleration.**

Move the gyroscopic term to the other side:

```math
J\dot{\boldsymbol{\omega}}
=
\boldsymbol{\tau}
-
\boldsymbol{\omega}\times(J\boldsymbol{\omega})
```

Multiply both sides by the inverse inertia matrix:

```math
J^{-1}J\dot{\boldsymbol{\omega}}
=
J^{-1}
\left[
\boldsymbol{\tau}
-
\boldsymbol{\omega}\times(J\boldsymbol{\omega})
\right]
```

Since the inverse inertia matrix multiplied by the inertia matrix gives the identity matrix:

```math
J^{-1}J=I
```

The final equation becomes:

```math
\boxed{
\dot{\boldsymbol{\omega}}
=
J^{-1}
\left[
\boldsymbol{\tau}
-
\boldsymbol{\omega}\times(J\boldsymbol{\omega})
\right]
}
```

This is Equation (9).

### What Does Equation (9) Do?

Equation (9) calculates the angular acceleration of the quadrotor based on the applied torque, its rotational inertia, and the coupling between its rotational axes.

**Control torque:** Produces rotational acceleration.

**Moment of inertia:** Determines how strongly the quadrotor resists angular acceleration.

**Gyroscopic coupling:** Accounts for the interaction between angular velocity and angular momentum.

**Angular acceleration:** Describes how angular velocity changes over time.

### Why Do We Use This Equation in Our Project?

The backstepping controller must calculate the torque required to make the quadrotor follow its desired angular velocity and attitude.

Equation (9) provides the mathematical relationship between the control torque and angular acceleration. The controller uses this relationship to compensate for the nonlinear rotational dynamics and determine the required torque.

Equation (8) describes how angular velocity changes the orientation, whereas Equation (9) describes how torque changes angular velocity.





## Equation (10): Quaternion Attitude Error

### Equation

The quaternion attitude error is defined as:

```math
\boxed{
q_{\mathrm{err}}
=
q_{\mathrm{ref}}^*\otimes q
}
```

### Meaning of the Variables

The actual attitude quaternion is:

```math
q=
\begin{bmatrix}
q_0\\
q_1\\
q_2\\
q_3
\end{bmatrix}
```

The desired attitude quaternion is:

```math
q_{\mathrm{ref}}=
\begin{bmatrix}
q_{\mathrm{ref},0}\\
q_{\mathrm{ref},1}\\
q_{\mathrm{ref},2}\\
q_{\mathrm{ref},3}
\end{bmatrix}
```

The conjugate of the reference quaternion is:

```math
q_{\mathrm{ref}}^*=
\begin{bmatrix}
q_{\mathrm{ref},0}\\
-q_{\mathrm{ref},1}\\
-q_{\mathrm{ref},2}\\
-q_{\mathrm{ref},3}
\end{bmatrix}
```

The symbol for quaternion multiplication is:

```math
\otimes
```

### Derivation of Equation (10)

**Step 1: Identify the actual and desired orientations.**

The quadrotor has an actual orientation represented by the quaternion:

```math
q
```

The controller also receives the desired orientation, represented by:

```math
q_{\mathrm{ref}}
```

Our objective is to determine the relative rotation between these two orientations.

**Step 2: Find the inverse of the desired quaternion.**

To calculate the relative orientation, we first need to reverse the desired rotation.

For a unit quaternion, the inverse equals its conjugate:

```math
q_{\mathrm{ref}}^{-1}=q_{\mathrm{ref}}^*
```

The conjugate is obtained by keeping the scalar component unchanged and reversing the signs of the three vector components.

**Step 3: Calculate the relative orientation.**

Quaternion multiplication allows us to combine rotations.

To express the actual orientation relative to the desired orientation, we multiply the inverse of the reference quaternion by the actual quaternion:

```math
q_{\mathrm{err}}
=
q_{\mathrm{ref}}^{-1}\otimes q
```

Since the reference quaternion is a unit quaternion:

```math
q_{\mathrm{ref}}^{-1}=q_{\mathrm{ref}}^*
```

Substituting this into the equation gives:

```math
q_{\mathrm{err}}
=
q_{\mathrm{ref}}^*\otimes q
```

Therefore, Equation (10) is:

```math
\boxed{
q_{\mathrm{err}}
=
q_{\mathrm{ref}}^*\otimes q
}
```

### Verification: What Happens When There Is No Attitude Error?

Suppose the actual attitude exactly matches the desired attitude:

```math
q=q_{\mathrm{ref}}
```

Substituting this into Equation (10):

```math
q_{\mathrm{err}}
=
q_{\mathrm{ref}}^*\otimes q_{\mathrm{ref}}
```

For a unit quaternion, multiplying its conjugate by itself produces the identity quaternion:

```math
q_{\mathrm{ref}}^*\otimes q_{\mathrm{ref}}
=
\begin{bmatrix}
1\\
0\\
0\\
0
\end{bmatrix}
```

Therefore:

```math
\boxed{
q_{\mathrm{err}}
=
\begin{bmatrix}
1\\
0\\
0\\
0
\end{bmatrix}
}
```

This confirms that the quaternion error becomes the identity quaternion when the actual and desired orientations are identical.

### What Does Equation (10) Do?

Equation (10) calculates the relative orientation between the actual quadrotor attitude and the desired attitude.

The controller uses this error to determine how the drone must rotate to reach its target orientation.

For example, if the desired orientation is different from the actual orientation, the error quaternion represents the relative rotation needed to describe that mismatch.

When the error quaternion approaches the identity quaternion, the actual and desired orientations align.

### Why Do We Use This Equation in Our Project?

Our quaternion-based backstepping controller needs a mathematical measure of the difference between the actual and desired orientations.

Equation (10) provides that measure. The controller uses the quaternion error to generate the desired angular-velocity correction in the next stage.

In summary:

- The reference quaternion represents the desired orientation.
- The actual quaternion represents the current orientation.
- The quaternion error represents their relative orientation.
- The identity quaternion indicates that the two orientations are aligned.

The next equation defines the angular-velocity tracking error, which measures the difference between the actual and desired rotational velocities.








## Equation (11): Angular Velocity Tracking Error

### Equation

The angular velocity tracking error is defined as:

```math
\boxed{
\boldsymbol{\omega}_{\mathrm{err}}
=
\boldsymbol{\omega}
-
\boldsymbol{\omega}_{\mathrm{ref}}
}
```

### Meaning of the Variables

The actual angular velocity is:

```math
\boldsymbol{\omega}
=
\begin{bmatrix}
\omega_x\\
\omega_y\\
\omega_z
\end{bmatrix}
```

The desired angular velocity is:

```math
\boldsymbol{\omega}_{\mathrm{ref}}
=
\begin{bmatrix}
\omega_{\mathrm{ref},x}\\
\omega_{\mathrm{ref},y}\\
\omega_{\mathrm{ref},z}
\end{bmatrix}
```

The angular velocity tracking error is:

```math
\boldsymbol{\omega}_{\mathrm{err}}
=
\begin{bmatrix}
\omega_{\mathrm{err},x}\\
\omega_{\mathrm{err},y}\\
\omega_{\mathrm{err},z}
\end{bmatrix}
```

### Derivation of Equation (11)

**Step 1: Identify the actual angular velocity.**

The actual angular velocity represents how quickly the quadrotor rotates about its three body axes.

```math
\boldsymbol{\omega}
=
\begin{bmatrix}
\omega_x\\
\omega_y\\
\omega_z
\end{bmatrix}
```

**Step 2: Identify the desired angular velocity.**

The reference trajectory specifies the angular velocity that the quadrotor should achieve.

```math
\boldsymbol{\omega}_{\mathrm{ref}}
=
\begin{bmatrix}
\omega_{\mathrm{ref},x}\\
\omega_{\mathrm{ref},y}\\
\omega_{\mathrm{ref},z}
\end{bmatrix}
```

**Step 3: Calculate the difference between the actual and desired values.**

Tracking error is defined as the actual value minus the desired value.

Therefore:

```math
\boldsymbol{\omega}_{\mathrm{err}}
=
\boldsymbol{\omega}
-
\boldsymbol{\omega}_{\mathrm{ref}}
```

Substituting the component vectors:

```math
\boldsymbol{\omega}_{\mathrm{err}}
=
\begin{bmatrix}
\omega_x\\
\omega_y\\
\omega_z
\end{bmatrix}
-
\begin{bmatrix}
\omega_{\mathrm{ref},x}\\
\omega_{\mathrm{ref},y}\\
\omega_{\mathrm{ref},z}
\end{bmatrix}
```

Subtracting the vectors component by component gives:

```math
\boldsymbol{\omega}_{\mathrm{err}}
=
\begin{bmatrix}
\omega_x-\omega_{\mathrm{ref},x}\\
\omega_y-\omega_{\mathrm{ref},y}\\
\omega_z-\omega_{\mathrm{ref},z}
\end{bmatrix}
```

Hence, Equation (11) is:

```math
\boxed{
\boldsymbol{\omega}_{\mathrm{err}}
=
\boldsymbol{\omega}
-
\boldsymbol{\omega}_{\mathrm{ref}}
}
```

### Example

Suppose the actual angular velocity is:

```math
\boldsymbol{\omega}
=
\begin{bmatrix}
2\\
1\\
3
\end{bmatrix}
```

and the desired angular velocity is:

```math
\boldsymbol{\omega}_{\mathrm{ref}}
=
\begin{bmatrix}
1\\
0.5\\
2
\end{bmatrix}
```

The angular velocity tracking error is:

```math
\boldsymbol{\omega}_{\mathrm{err}}
=
\begin{bmatrix}
2\\
1\\
3
\end{bmatrix}
-
\begin{bmatrix}
1\\
0.5\\
2
\end{bmatrix}
```

Therefore:

```math
\boxed{
\boldsymbol{\omega}_{\mathrm{err}}
=
\begin{bmatrix}
1\\
0.5\\
1
\end{bmatrix}
}
```

The positive values indicate that the actual angular velocity is greater than the desired angular velocity along all three axes in this example.

### What Does Equation (11) Do?

Equation (11) measures the difference between the actual rotational motion of the quadrotor and its desired rotational motion.

If the actual angular velocity is greater than the desired value along an axis, the error along that axis is positive. If it is smaller, the error is negative.

When the actual and desired angular velocities match:

```math
\boldsymbol{\omega}
=
\boldsymbol{\omega}_{\mathrm{ref}}
```

their difference becomes:

```math
\boxed{
\boldsymbol{\omega}_{\mathrm{err}}
=
\begin{bmatrix}
0\\
0\\
0
\end{bmatrix}
}
```

This is the desired angular velocity tracking condition.

### Why Do We Use This Equation in Our Project?

Our controller must track both the desired orientation and the desired rotational motion.

Equation (10) measures the attitude difference using quaternion error, while Equation (11) measures the angular velocity difference.

The angular velocity error is then used to derive the quaternion error dynamics and design the backstepping controller.

In summary, Equation (10) tells us how the orientations differ, whereas Equation (11) tells us how the rotational velocities differ.


## Equation (12): Quaternion Error Dynamics

### Equation

The quaternion error dynamics are given by:

```math
\boxed{
\dot{q}_{\mathrm{err}}
=
\frac{1}{2}
q_{\mathrm{err}}\otimes
\begin{bmatrix}
0\\
\boldsymbol{\omega}_{\mathrm{err}}
\end{bmatrix}
}
```

### Meaning of the Variables

The quaternion attitude error is:

```math
q_{\mathrm{err}}
=
q_{\mathrm{ref}}^*\otimes q
```

The angular velocity error is:

```math
\boldsymbol{\omega}_{\mathrm{err}}
=
\boldsymbol{\omega}
-
\boldsymbol{\omega}_{\mathrm{ref}}
```

The rate of change of the quaternion error is:

```math
\dot{q}_{\mathrm{err}}
```

### Derivation of Equation (12)

**Step 1: Start with the quaternion kinematic equation.**

From Equation (8), the actual quaternion dynamics are:

```math
\dot{q}
=
\frac{1}{2}
q\otimes
\begin{bmatrix}
0\\
\boldsymbol{\omega}
\end{bmatrix}
```

This equation describes how the actual orientation changes with angular velocity.

**Step 2: Define the quaternion attitude error.**

From Equation (10), the attitude error is:

```math
q_{\mathrm{err}}
=
q_{\mathrm{ref}}^*\otimes q
```

This represents the relative orientation between the desired and actual attitudes.

**Step 3: Define the angular velocity error.**

From Equation (11), the angular velocity error is:

```math
\boldsymbol{\omega}_{\mathrm{err}}
=
\boldsymbol{\omega}
-
\boldsymbol{\omega}_{\mathrm{ref}}
```

Rearranging:

```math
\boldsymbol{\omega}
=
\boldsymbol{\omega}_{\mathrm{err}}
+
\boldsymbol{\omega}_{\mathrm{ref}}
```

This expresses the actual angular velocity as the desired reference angular velocity plus the angular velocity tracking error.

**Step 4: Differentiate the quaternion error.**

Starting from Equation (10):

```math
q_{\mathrm{err}}
=
q_{\mathrm{ref}}^*\otimes q
```

Apply the product rule for quaternion multiplication:

```math
\dot{q}_{\mathrm{err}}
=
\dot{q}_{\mathrm{ref}}^*\otimes q
+
q_{\mathrm{ref}}^*\otimes\dot{q}
```

This is the general derivative of the defined quaternion error.

**Step 5: Express the error dynamics using the angular velocity error.**

Under the relative angular-velocity convention adopted by the research paper, the quaternion error dynamics are expressed as:

```math
\boxed{
\dot{q}_{\mathrm{err}}
=
\frac{1}{2}
q_{\mathrm{err}}\otimes
\begin{bmatrix}
0\\
\boldsymbol{\omega}_{\mathrm{err}}
\end{bmatrix}
}
```

This is Equation (12) as presented in the paper.

**Mathematical note:** The final step uses the paper's adopted relative-motion convention. In a general derivation using body-frame angular velocities, the product-rule expression does not automatically reduce to this equation; the relative angular velocity and reference-frame conventions must be consistent.

### What Does Equation (12) Do?

Equation (12) describes how the attitude error changes over time.

The quaternion error indicates the difference between the actual and desired orientations, while the angular velocity error determines how the relative attitude evolves under the adopted error model.

When the angular velocity error approaches zero, the relative quaternion becomes stationary under this equation. To ensure the attitude error converges to the identity quaternion, the controller must also be designed to drive the attitude error toward its target.

### Why Do We Use This Equation in Our Project?

Equation (12) establishes the relationship needed to design the first stage of the backstepping controller.

The controller uses the quaternion error to calculate a desired angular velocity error. This becomes the virtual control used in the next equation.

In summary:

- Equation (10) calculates the quaternion attitude error.
- Equation (11) calculates the angular velocity tracking error.
- Equation (12) describes the evolution of the quaternion error under the paper's adopted error model.

Equation (12) is the starting point for designing the virtual control in Equation (13).



## Equation (13): Virtual Control — Desired Angular Velocity Error

### Equation

The virtual control is defined as:

```math
\boxed{
\begin{bmatrix}
0\\
\boldsymbol{\omega}_{\mathrm{err},d}
\end{bmatrix}
=
-k_1 q_{\mathrm{err}}^*
\otimes
(q_{\mathrm{err}}-q_i)
}
```

where the identity quaternion is:

```math
q_i=
\begin{bmatrix}
1\\
0\\
0\\
0
\end{bmatrix}
```

The first control gain must satisfy:

```math
k_1>0
```

### Derivation of Equation (13)

**Step 1: Start with Equation (12).**

The quaternion error dynamics are:

```math
\dot q_{\mathrm{err}}
=
\frac{1}{2}
q_{\mathrm{err}}\otimes
\begin{bmatrix}
0\\
\boldsymbol{\omega}_{\mathrm{err}}
\end{bmatrix}
```

This equation shows that the attitude error changes according to the angular velocity error.

Our objective is to make the quaternion error approach the identity quaternion.

**Step 2: Choose the desired error convergence behaviour.**

We want the quaternion error to move toward the identity quaternion:

```math
q_{\mathrm{err}}\rightarrow q_i
```

The paper selects the following desired error dynamics:

```math
\dot q_{\mathrm{err}}
=
-\frac{k_1}{2}(q_{\mathrm{err}}-q_i)
```

The negative sign makes the feedback act in the direction that reduces the error under the intended control law.

**Step 3: Assume the actual angular velocity error follows the desired virtual control.**

At the ideal first backstepping stage:

```math
\boldsymbol{\omega}_{\mathrm{err}}
=
\boldsymbol{\omega}_{\mathrm{err},d}
```

Substituting this into Equation (12):

```math
\dot q_{\mathrm{err}}
=
\frac{1}{2}
q_{\mathrm{err}}\otimes
\begin{bmatrix}
0\\
\boldsymbol{\omega}_{\mathrm{err},d}
\end{bmatrix}
```

**Step 4: Solve for the desired angular velocity error.**

Using the desired error dynamics:

```math
-\frac{k_1}{2}(q_{\mathrm{err}}-q_i)
=
\frac{1}{2}
q_{\mathrm{err}}\otimes
\begin{bmatrix}
0\\
\boldsymbol{\omega}_{\mathrm{err},d}
\end{bmatrix}
```

Multiply both sides by two:

```math
-k_1(q_{\mathrm{err}}-q_i)
=
q_{\mathrm{err}}\otimes
\begin{bmatrix}
0\\
\boldsymbol{\omega}_{\mathrm{err},d}
\end{bmatrix}
```

Multiply on the left by the conjugate quaternion:

```math
-k_1q_{\mathrm{err}}^*
\otimes(q_{\mathrm{err}}-q_i)
=
q_{\mathrm{err}}^*\otimes q_{\mathrm{err}}
\otimes
\begin{bmatrix}
0\\
\boldsymbol{\omega}_{\mathrm{err},d}
\end{bmatrix}
```

For a unit quaternion:

```math
q_{\mathrm{err}}^*\otimes q_{\mathrm{err}}=q_i
```

Therefore, the intended virtual-control expression is:

```math
\begin{bmatrix}
0\\
\boldsymbol{\omega}_{\mathrm{err},d}
\end{bmatrix}
=
-k_1q_{\mathrm{err}}^*
\otimes(q_{\mathrm{err}}-q_i)
```

This is Equation (13) as presented in the paper.

**Mathematical clarification:** Because angular velocity is a three-dimensional vector, the right-hand side must be interpreted using its vector part. If the error quaternion is written as:

```math
q_{\mathrm{err}}
=
\begin{bmatrix}
\eta\\
\boldsymbol{\epsilon}
\end{bmatrix}
```

then the vector-part result is:

```math
\boxed{
\boldsymbol{\omega}_{\mathrm{err},d}
=
-k_1\boldsymbol{\epsilon}
}
```

This is the corresponding virtual angular-velocity command for the vector-part implementation.

### What Does Equation (13) Do?

Equation (13) converts the quaternion attitude error into a desired angular velocity error.

The controller uses this virtual control to determine the rotational correction required to reduce the attitude mismatch.

The gain controls the strength of this correction:

```math
k_1>0
```

A larger gain increases the corrective command for a given error, although excessively large gains may cause numerical or actuator-related problems in an implementation.

### Why Do We Use Equation (13)?

The actual attitude cannot be corrected directly by applying an orientation value as torque. Instead, the controller first calculates a desired angular velocity correction and then uses the torque controller to make the drone follow that correction.

Equation (13) therefore forms the first stage of the backstepping controller.

---

## Equation (14): Desired Quaternion Error Dynamics

### Equation

The paper gives the desired quaternion error dynamics as:

```math
\boxed{
\dot q_{\mathrm{err}}
=
-\frac{k_1}{2}(q_{\mathrm{err}}-q_i)
}
```

### Derivation of Equation (14)

**Step 1: Start with Equation (12).**

```math
\dot q_{\mathrm{err}}
=
\frac{1}{2}
q_{\mathrm{err}}\otimes
\begin{bmatrix}
0\\
\boldsymbol{\omega}_{\mathrm{err}}
\end{bmatrix}
```

**Step 2: Assume ideal tracking of the virtual control.**

```math
\boldsymbol{\omega}_{\mathrm{err}}
=
\boldsymbol{\omega}_{\mathrm{err},d}
```

Therefore:

```math
\dot q_{\mathrm{err}}
=
\frac{1}{2}
q_{\mathrm{err}}\otimes
\begin{bmatrix}
0\\
\boldsymbol{\omega}_{\mathrm{err},d}
\end{bmatrix}
```

**Step 3: Substitute the intended virtual-control expression from Equation (13).**

```math
\dot q_{\mathrm{err}}
=
-\frac{k_1}{2}
q_{\mathrm{err}}\otimes
q_{\mathrm{err}}^*
\otimes
(q_{\mathrm{err}}-q_i)
```

**Step 4: Use the unit-quaternion identity.**

```math
q_{\mathrm{err}}\otimes q_{\mathrm{err}}^*=q_i
```

Consequently, the paper's intended simplification is:

```math
\dot q_{\mathrm{err}}
=
-\frac{k_1}{2}(q_{\mathrm{err}}-q_i)
```

This is Equation (14) as presented in the paper.

### What Does Equation (14) Do?

The equation specifies the desired behaviour of the quaternion error. Its purpose is to drive the attitude error toward the identity quaternion.

The intended target is:

```math
q_{\mathrm{err}}\rightarrow
\begin{bmatrix}
1\\
0\\
0\\
0
\end{bmatrix}
```

When the error approaches the identity quaternion, the relative orientation between the actual drone and the desired orientation approaches zero rotation.



---

## Equation (15): Angular Velocity Error Dynamics

### Equation

The angular velocity error dynamics are defined as:

```math
\boxed{
\dot{\boldsymbol{\omega}}_{\mathrm{err}}
=
\dot{\boldsymbol{\omega}}
-
\dot{\boldsymbol{\omega}}_{\mathrm{ref}}
}
```

### Meaning of the Variables

The angular velocity error is:

```math
\boldsymbol{\omega}_{\mathrm{err}}
=
\boldsymbol{\omega}
-
\boldsymbol{\omega}_{\mathrm{ref}}
```

The actual angular acceleration is:

```math
\dot{\boldsymbol{\omega}}
```

The reference angular acceleration is:

```math
\dot{\boldsymbol{\omega}}_{\mathrm{ref}}
```

### Derivation of Equation (15)

**Step 1: Start with Equation (11).**

The angular velocity tracking error is:

```math
\boldsymbol{\omega}_{\mathrm{err}}
=
\boldsymbol{\omega}
-
\boldsymbol{\omega}_{\mathrm{ref}}
```

This represents the difference between the actual angular velocity and the desired angular velocity.

**Step 2: Differentiate both sides with respect to time.**

```math
\frac{d\boldsymbol{\omega}_{\mathrm{err}}}{dt}
=
\frac{d}{dt}
\left(
\boldsymbol{\omega}
-
\boldsymbol{\omega}_{\mathrm{ref}}
\right)
```

Using the differentiation rule for the difference of two time-dependent vectors:

```math
\frac{d\boldsymbol{\omega}_{\mathrm{err}}}{dt}
=
\frac{d\boldsymbol{\omega}}{dt}
-
\frac{d\boldsymbol{\omega}_{\mathrm{ref}}}{dt}
```

**Step 3: Express the derivatives as angular accelerations.**

By definition:

```math
\frac{d\boldsymbol{\omega}_{\mathrm{err}}}{dt}
=
\dot{\boldsymbol{\omega}}_{\mathrm{err}}
```

Similarly:

```math
\frac{d\boldsymbol{\omega}}{dt}
=
\dot{\boldsymbol{\omega}}
```

and:

```math
\frac{d\boldsymbol{\omega}_{\mathrm{ref}}}{dt}
=
\dot{\boldsymbol{\omega}}_{\mathrm{ref}}
```

Substituting these definitions gives:

```math
\boxed{
\dot{\boldsymbol{\omega}}_{\mathrm{err}}
=
\dot{\boldsymbol{\omega}}
-
\dot{\boldsymbol{\omega}}_{\mathrm{ref}}
}
```

This is Equation (15).

### What Does Equation (15) Do?

Equation (15) describes how the angular velocity tracking error changes over time.

If the actual angular acceleration is different from the reference angular acceleration, the angular velocity error changes accordingly.

If the two angular accelerations are equal:

```math
\dot{\boldsymbol{\omega}}
=
\dot{\boldsymbol{\omega}}_{\mathrm{ref}}
```

then:

```math
\dot{\boldsymbol{\omega}}_{\mathrm{err}}=0
```

This means the angular velocity error is momentarily constant. It does not necessarily mean the error itself is zero.

### Why Do We Use Equation (15)?

The backstepping controller needs the derivative of angular velocity error to calculate the derivative of its next error variable.

Equation (15) supplies this relationship. In the following controller-design steps, the actual angular acceleration is replaced using Equation (9), allowing the control torque to be determined.

In summary, Equation (13) defines the virtual angular-velocity correction, Equation (14) describes the intended attitude-error convergence behaviour, and Equation (15) provides the derivative needed for the second backstepping stage.








## Equation (16): Backstepping Error

### Equation

The backstepping error is defined as:

```math
\boxed{
\boldsymbol{\delta}
=
\boldsymbol{\omega}_{\mathrm{err}}
-
\boldsymbol{\omega}_{\mathrm{err},d}
}
```

### Meaning of the Variables

The actual angular velocity tracking error is:

```math
\boldsymbol{\omega}_{\mathrm{err}}
```

The desired angular velocity error generated by the virtual controller is:

```math
\boldsymbol{\omega}_{\mathrm{err},d}
```

Their difference is the backstepping error:

```math
\boldsymbol{\delta}
```

### Derivation of Equation (16)

**Step 1: Recall Equation (11).**

The angular velocity tracking error is:

```math
\boldsymbol{\omega}_{\mathrm{err}}
=
\boldsymbol{\omega}
-
\boldsymbol{\omega}_{\mathrm{ref}}
```

This tells us the difference between the actual angular velocity and the reference angular velocity.

**Step 2: Identify the virtual control.**

Equation (13) defines a desired angular velocity error:

```math
\boldsymbol{\omega}_{\mathrm{err},d}
```

This is the angular velocity correction that the first stage of the controller wants the system to follow.

**Step 3: Calculate the difference.**

The backstepping approach defines the difference between the actual error and the desired error as:

```math
\boxed{
\boldsymbol{\delta}
=
\boldsymbol{\omega}_{\mathrm{err}}
-
\boldsymbol{\omega}_{\mathrm{err},d}
}
```

Rearranging gives:

```math
\boxed{
\boldsymbol{\omega}_{\mathrm{err}}
=
\boldsymbol{\omega}_{\mathrm{err},d}
+
\boldsymbol{\delta}
}
```

This is Equation (16).

### What Does Equation (16) Do?

It measures how far the actual angular velocity error is from the desired angular velocity error.

If the backstepping error becomes zero:

```math
\boldsymbol{\delta}=\mathbf{0}
```

then:

```math
\boldsymbol{\omega}_{\mathrm{err}}
=
\boldsymbol{\omega}_{\mathrm{err},d}
```

The actual angular velocity error therefore matches the virtual control.

### Why Do We Use Equation (16)?

The actual attitude cannot be controlled directly by assigning a desired orientation. The controller first calculates the desired angular velocity correction and then generates torque to make the drone follow it.

Equation (16) defines the error that the second stage of backstepping must drive toward zero.

---

## Equation (17): Quaternion Error Dynamics with Backstepping Error

### Equation

The paper expresses the quaternion error dynamics as:

```math
\boxed{
\dot q_{\mathrm{err}}
=
-\frac{k_1}{2}
(q_{\mathrm{err}}-q_i)
+
\frac{1}{2}
q_{\mathrm{err}}\otimes
\begin{bmatrix}
0\\
\boldsymbol{\delta}
\end{bmatrix}
}
```

where:

```math
q_i=
\begin{bmatrix}
1\\
0\\
0\\
0
\end{bmatrix}
```

### Derivation of Equation (17)

**Step 1: Start with Equation (12).**

The quaternion error dynamics are:

```math
\dot q_{\mathrm{err}}
=
\frac{1}{2}
q_{\mathrm{err}}\otimes
\begin{bmatrix}
0\\
\boldsymbol{\omega}_{\mathrm{err}}
\end{bmatrix}
```

**Step 2: Use Equation (16).**

From the backstepping error definition:

```math
\boldsymbol{\delta}
=
\boldsymbol{\omega}_{\mathrm{err}}
-
\boldsymbol{\omega}_{\mathrm{err},d}
```

Rearranging:

```math
\boldsymbol{\omega}_{\mathrm{err}}
=
\boldsymbol{\omega}_{\mathrm{err},d}
+
\boldsymbol{\delta}
```

**Step 3: Substitute into Equation (12).**

```math
\dot q_{\mathrm{err}}
=
\frac{1}{2}
q_{\mathrm{err}}\otimes
\begin{bmatrix}
0\\
\boldsymbol{\omega}_{\mathrm{err},d}
+
\boldsymbol{\delta}
\end{bmatrix}
```

By the linearity of quaternion multiplication in its second operand:

```math
\begin{aligned}
\dot q_{\mathrm{err}}
={}&
\frac{1}{2}
q_{\mathrm{err}}\otimes
\begin{bmatrix}
0\\
\boldsymbol{\omega}_{\mathrm{err},d}
\end{bmatrix}\\
&+
\frac{1}{2}
q_{\mathrm{err}}\otimes
\begin{bmatrix}
0\\
\boldsymbol{\delta}
\end{bmatrix}
\end{aligned}
```

**Step 4: Substitute the virtual-control convergence term.**

The paper intends the first term to provide the desired attitude-error feedback:

```math
-\frac{k_1}{2}(q_{\mathrm{err}}-q_i)
```

Using that intended substitution gives:

```math
\boxed{
\dot q_{\mathrm{err}}
=
-\frac{k_1}{2}
(q_{\mathrm{err}}-q_i)
+
\frac{1}{2}
q_{\mathrm{err}}\otimes
\begin{bmatrix}
0\\
\boldsymbol{\delta}
\end{bmatrix}
}
```

This is Equation (17) as presented by the paper's design approach.

### What Does Equation (17) Do?

It shows that the attitude error depends on two contributions:

1. The desired corrective feedback, intended to reduce the quaternion error.
2. The remaining backstepping error, which represents the mismatch between actual and desired angular velocity errors.

When the backstepping error approaches zero, the second contribution disappears from this equation.

### Why Do We Use Equation (17)?

It connects the first backstepping stage to the second. It allows the controller design to account for the fact that the physical angular velocity may not instantly follow the virtual control.

**Mathematical clarification:** The paper's simplification of the full quaternion feedback is not an exact identity for a general unit quaternion when angular velocity is restricted to three components. In a strictly consistent implementation, the vector-part virtual control and quaternion kinematics must be used together.

---

## Equation (18): Backstepping Error Dynamics

### Equation

The derivative of the backstepping error is:

```math
\boxed{
\dot{\boldsymbol{\delta}}
=
\dot{\boldsymbol{\omega}}_{\mathrm{err}}
-
\dot{\boldsymbol{\omega}}_{\mathrm{err},d}
}
```

### Derivation of Equation (18)

**Step 1: Start with Equation (16).**

```math
\boldsymbol{\delta}
=
\boldsymbol{\omega}_{\mathrm{err}}
-
\boldsymbol{\omega}_{\mathrm{err},d}
```

**Step 2: Differentiate both sides with respect to time.**

```math
\frac{d\boldsymbol{\delta}}{dt}
=
\frac{d}{dt}
\left(
\boldsymbol{\omega}_{\mathrm{err}}
-
\boldsymbol{\omega}_{\mathrm{err},d}
\right)
```

Using the differentiation rule for the difference of two vectors:

```math
\frac{d\boldsymbol{\delta}}{dt}
=
\frac{d\boldsymbol{\omega}_{\mathrm{err}}}{dt}
-
\frac{d\boldsymbol{\omega}_{\mathrm{err},d}}{dt}
```

**Step 3: Express the derivatives in dot notation.**

```math
\boxed{
\dot{\boldsymbol{\delta}}
=
\dot{\boldsymbol{\omega}}_{\mathrm{err}}
-
\dot{\boldsymbol{\omega}}_{\mathrm{err},d}
}
```

This is Equation (18).

### What Does Equation (18) Do?

It describes how quickly the backstepping error changes over time.

The error changes according to the difference between the derivative of the actual angular velocity error and the derivative of the desired angular velocity error.

### Why Do We Use Equation (18)?

To design the torque controller, we need an equation describing how the backstepping error evolves under the applied torque.

Equation (18) provides the starting point for deriving Equation (19).

---

## Equation (19): Backstepping Error Dynamics Including Rotational Dynamics

### Equation

The backstepping error dynamics become:

```math
\boxed{
\dot{\boldsymbol{\delta}}
=
J^{-1}
\left[
\boldsymbol{\tau}
-
\boldsymbol{\omega}\times(J\boldsymbol{\omega})
\right]
-
\dot{\boldsymbol{\omega}}_{\mathrm{ref}}
-
\dot{\boldsymbol{\omega}}_{\mathrm{err},d}
}
```

### Derivation of Equation (19)

**Step 1: Start with Equation (18).**

```math
\dot{\boldsymbol{\delta}}
=
\dot{\boldsymbol{\omega}}_{\mathrm{err}}
-
\dot{\boldsymbol{\omega}}_{\mathrm{err},d}
```

**Step 2: Substitute Equation (15).**

The angular velocity error dynamics are:

```math
\dot{\boldsymbol{\omega}}_{\mathrm{err}}
=
\dot{\boldsymbol{\omega}}
-
\dot{\boldsymbol{\omega}}_{\mathrm{ref}}
```

Substituting:

```math
\dot{\boldsymbol{\delta}}
=
\dot{\boldsymbol{\omega}}
-
\dot{\boldsymbol{\omega}}_{\mathrm{ref}}
-
\dot{\boldsymbol{\omega}}_{\mathrm{err},d}
```

**Step 3: Substitute Equation (9).**

The rotational dynamics are:

```math
\dot{\boldsymbol{\omega}}
=
J^{-1}
\left[
\boldsymbol{\tau}
-
\boldsymbol{\omega}\times(J\boldsymbol{\omega})
\right]
```

Replacing the angular acceleration gives:

```math
\boxed{
\dot{\boldsymbol{\delta}}
=
J^{-1}
\left[
\boldsymbol{\tau}
-
\boldsymbol{\omega}\times(J\boldsymbol{\omega})
\right]
-
\dot{\boldsymbol{\omega}}_{\mathrm{ref}}
-
\dot{\boldsymbol{\omega}}_{\mathrm{err},d}
}
```

This is Equation (19).

### What Does Equation (19) Do?

This equation connects the backstepping error directly to the physical torque input.

It contains:

- The applied control torque.
- The nonlinear gyroscopic coupling.
- The reference angular acceleration.
- The derivative of the virtual-control command.

### Why Do We Use Equation (19)?

Equation (19) is the main equation used to design the torque controller. By choosing the torque appropriately, the controller can cancel the unwanted dynamics and impose a desired convergence law on the backstepping error.

---

## Equation (20): Derivative of the Virtual Control

### Equation

The derivative of the desired angular velocity error is calculated from the virtual-control expression:

```math
\boxed{
\begin{bmatrix}
0\\
\dot{\boldsymbol{\omega}}_{\mathrm{err},d}
\end{bmatrix}
=
\frac{d}{dt}
\left[
-k_1q_{\mathrm{err}}^*
\otimes(q_{\mathrm{err}}-q_i)
\right]
}
```

For a mathematically consistent implementation, the vector part of the right-hand side is used to obtain the derivative of the three-dimensional angular-velocity command.

### Derivation of Equation (20)

**Step 1: Start with Equation (13).**

The paper's virtual-control expression is:

```math
\begin{bmatrix}
0\\
\boldsymbol{\omega}_{\mathrm{err},d}
\end{bmatrix}
=
-k_1q_{\mathrm{err}}^*
\otimes(q_{\mathrm{err}}-q_i)
```

**Step 2: Differentiate both sides.**

```math
\frac{d}{dt}
\begin{bmatrix}
0\\
\boldsymbol{\omega}_{\mathrm{err},d}
\end{bmatrix}
=
\frac{d}{dt}
\left[
-k_1q_{\mathrm{err}}^*
\otimes(q_{\mathrm{err}}-q_i)
\right]
```

Assuming the gain is constant:

```math
\frac{d}{dt}
\left[
-k_1q_{\mathrm{err}}^*
\otimes(q_{\mathrm{err}}-q_i)
\right]
=
-k_1
\frac{d}{dt}
\left[
q_{\mathrm{err}}^*
\otimes(q_{\mathrm{err}}-q_i)
\right]
```

**Step 3: Apply the product rule.**

The derivative of a product of time-dependent quaternions follows the product rule:

```math
\begin{aligned}
\dot{\boldsymbol{\omega}}_{\mathrm{err},d}
\ \text{(quaternion form)}
={}&-k_1\Big[
\dot q_{\mathrm{err}}^*
\otimes(q_{\mathrm{err}}-q_i)\\
&+
q_{\mathrm{err}}^*\otimes\dot q_{\mathrm{err}}
\Big]
\end{aligned}
```

In this expression, the quaternion components and their derivatives are obtained from the quaternion error dynamics.

The vector component supplies the angular-velocity derivative needed by the torque controller.

### What Does Equation (20) Do?

It calculates how quickly the virtual angular-velocity command is changing.

If the attitude error changes rapidly, the virtual angular-velocity command may also change rapidly. The torque controller must account for this change.

### Why Do We Use Equation (20)?

Equation (19) contains the derivative of the virtual control. Equation (20) provides the method for obtaining it from the quaternion error and its derivative.

This term is required to compensate for the time-varying virtual control in the backstepping design.

---

## Equation (21): Dynamics Compensation Torque

### Equation

The first component of the control torque is defined as:

```math
\boxed{
\boldsymbol{\tau}_1
=
\boldsymbol{\omega}\times(J\boldsymbol{\omega})
+
J\dot{\boldsymbol{\omega}}_{\mathrm{ref}}
+
J\dot{\boldsymbol{\omega}}_{\mathrm{err},d}
}
```

### Derivation of Equation (21)

**Step 1: Start with Equation (19).**

```math
\dot{\boldsymbol{\delta}}
=
J^{-1}
\left[
\boldsymbol{\tau}
-
\boldsymbol{\omega}\times(J\boldsymbol{\omega})
\right]
-
\dot{\boldsymbol{\omega}}_{\mathrm{ref}}
-
\dot{\boldsymbol{\omega}}_{\mathrm{err},d}
```

**Step 2: Divide the torque into two components.**

The paper proposes:

```math
\boldsymbol{\tau}
=
\boldsymbol{\tau}_1+\boldsymbol{\tau}_2
```

The purpose of the first component is to cancel the nonlinear dynamics and reference-acceleration terms.

**Step 3: Cancel the gyroscopic term.**

The rotational model contains:

```math
-\boldsymbol{\omega}\times(J\boldsymbol{\omega})
```

inside the torque-dependent expression. Choosing the compensation torque to include:

```math
\boldsymbol{\omega}\times(J\boldsymbol{\omega})
```

cancels this term.

**Step 4: Cancel the reference acceleration terms.**

The error dynamics also contain:

```math
-\dot{\boldsymbol{\omega}}_{\mathrm{ref}}
```

and:

```math
-\dot{\boldsymbol{\omega}}_{\mathrm{err},d}
```

Multiplying these acceleration terms by the inertia matrix when constructing the torque cancels them after substitution into the rotational dynamics.

Therefore, choose:

```math
\boxed{
\boldsymbol{\tau}_1
=
\boldsymbol{\omega}\times(J\boldsymbol{\omega})
+
J\dot{\boldsymbol{\omega}}_{\mathrm{ref}}
+
J\dot{\boldsymbol{\omega}}_{\mathrm{err},d}
}
```

This is Equation (21).

### What Does Equation (21) Do?

The first torque component compensates for:

1. The gyroscopic coupling between rotational axes.
2. The angular acceleration requested by the reference trajectory.
3. The changing virtual angular-velocity command.

### Why Do We Use Equation (21)?

These terms are part of the quadrotor's nonlinear rotational model. Compensating for them simplifies the remaining backstepping error dynamics, allowing the second torque component to focus on reducing the error.

---

## Equation (22): Fractional-Power Feedback Torque

### Equation

The second component of the control torque is:

```math
\boxed{
\boldsymbol{\tau}_2
=
-k_2J\boldsymbol{\delta}^{1/3}
}
```

where the second control gain satisfies:

```math
k_2>0
```

The fractional power is applied componentwise using the real cube root.

### Derivation of Equation (22)

**Step 1: Identify the remaining error dynamics.**

Once the compensation torque in Equation (21) cancels the nonlinear and reference terms, Equation (19) becomes:

```math
\dot{\boldsymbol{\delta}}
=
J^{-1}\boldsymbol{\tau}_2
```

**Step 2: Choose the desired error convergence law.**

The controller aims to drive the backstepping error to zero.

The paper selects the nonlinear convergence law:

```math
\dot{\boldsymbol{\delta}}
=
-k_2\boldsymbol{\delta}^{1/3}
```

**Step 3: Solve for the torque component.**

Multiply both sides by the inertia matrix:

```math
J\dot{\boldsymbol{\delta}}
=
-k_2J\boldsymbol{\delta}^{1/3}
```

Since the desired convergence law specifies the derivative of the error, the torque required to implement it is:

```math
\boxed{
\boldsymbol{\tau}_2
=
-k_2J\boldsymbol{\delta}^{1/3}
}
```

This is Equation (22).

### What Does Equation (22) Do?

It generates a torque correction based on the backstepping error. Its negative sign ensures that each error component receives feedback directed toward zero.

The gain controls the strength of the correction:

```math
k_2>0
```

A larger gain generally gives stronger correction, subject to actuator limits and numerical stability.

---

## Why Do We Use the Power of One-Third?

The second backstepping controller uses the fractional-power feedback law:

```math
\boxed{
\dot{\boldsymbol{\delta}}
=
-k_2\boldsymbol{\delta}^{1/3}
}
```

The exponent is one-third. To understand why it is used, consider one component of the backstepping error.

### 1. Understand the Scalar Error Equation

For one error component, the equation becomes:

```math
\boxed{
\dot{\delta}
=
-k_2\sqrt[3]{\delta}
}
```

Here, the cube root is the real cube root. It preserves the sign of the input.

For a positive error:

```math
\delta>0
\quad\Longrightarrow\quad
\sqrt[3]{\delta}>0
```

Therefore:

```math
\dot{\delta}<0
```

The error decreases toward zero.

For a negative error:

```math
\delta<0
\quad\Longrightarrow\quad
\sqrt[3]{\delta}<0
```

Therefore:

```math
\dot{\delta}>0
```

The error increases toward zero.

In both cases, the feedback drives the error toward zero.

### 2. Derive the Finite-Time Convergence

Consider a positive initial error. The differential equation is:

```math
\dot{\delta}=-k_2\delta^{1/3}
```

Write the derivative explicitly:

```math
\frac{d\delta}{dt}
=
-k_2\delta^{1/3}
```

Separate the variables:

```math
\delta^{-1/3}\,d\delta
=
-k_2\,dt
```

Integrate both sides:

```math
\int_{\delta(0)}^{0}
\delta^{-1/3}\,d\delta
=
-k_2\int_0^T dt
```

The integral of the fractional power is:

```math
\int\delta^{-1/3}\,d\delta
=
\frac{3}{2}\delta^{2/3}
```

Substituting the integral:

```math
\frac{3}{2}
\left[
0^{2/3}-\delta(0)^{2/3}
\right]
=
-k_2T
```

Simplifying:

```math
-\frac{3}{2}\delta(0)^{2/3}
=
-k_2T
```

Cancel the negative signs:

```math
\frac{3}{2}\delta(0)^{2/3}
=
k_2T
```

Therefore, the time required for the error to reach zero is:

```math
\boxed{
T=
\frac{3}{2k_2}
|\delta(0)|^{2/3}
}
```

The absolute value makes the result valid for either sign of the initial error.

This is the ideal convergence time for the scalar error equation.

### 3. Why Not Use a Power of One?

With linear feedback, the error equation would be:

```math
\dot{\delta}=-k_2\delta
```

Its solution is:

```math
\delta(t)=\delta(0)e^{-k_2t}
```

The error approaches zero exponentially but does not reach exactly zero in finite time under the ideal continuous-time model.

With the one-third power:

```math
\dot{\delta}=-k_2\sqrt[3]{\delta}
```

the error reaches zero in finite time for any finite initial error.

### 4. Is One-Third the Only Possible Exponent?

No. More generally, a fractional-power feedback law can be written as:

```math
\dot{\delta}=-k_2\delta^p,\qquad \delta>0
```

```math
\dot{\delta}=k_2(-\delta)^p,\qquad \delta<0
```

For this project, however, we use the exponent:

```math
p=\frac{1}{3}
```

The one-third exponent is the particular choice used in the research paper. It provides finite-time convergence for the ideal error dynamics.

### 5. Why Is This Important in Our Drone Project?

The purpose of the one-third power is to reduce the backstepping error and make it reach zero in finite time under the assumed mathematical model.

The feedback torque is:

```math
\boxed{
\boldsymbol{\tau}_2
=
-k_2J\boldsymbol{\delta}^{1/3}
}
```

The complete controller combines the compensation torque with this feedback torque:

```math
\boldsymbol{\tau}
=
\boldsymbol{\tau}_1+\boldsymbol{\tau}_2
```

The compensation torque handles the rotational dynamics, while the fractional-power feedback drives the remaining error toward zero.

In an actual simulation, numerical integration, measurement noise, disturbances, and actuator limits can affect the observed convergence.

---

## Equation (23): Final Backstepping Error Dynamics

### Equation

The resulting error dynamics are:

```math
\boxed{
\dot{\boldsymbol{\delta}}
=
-k_2\boldsymbol{\delta}^{1/3}
}
```

### Derivation of Equation (23)

**Step 1: Start with Equation (19).**

```math
\dot{\boldsymbol{\delta}}
=
J^{-1}
\left[
\boldsymbol{\tau}
-
\boldsymbol{\omega}\times(J\boldsymbol{\omega})
\right]
-
\dot{\boldsymbol{\omega}}_{\mathrm{ref}}
-
\dot{\boldsymbol{\omega}}_{\mathrm{err},d}
```

**Step 2: Substitute the torque decomposition.**

```math
\boldsymbol{\tau}
=
\boldsymbol{\tau}_1+\boldsymbol{\tau}_2
```

Therefore:

```math
\begin{aligned}
\dot{\boldsymbol{\delta}}
={}&
J^{-1}
\left[
\boldsymbol{\tau}_1+\boldsymbol{\tau}_2
-\boldsymbol{\omega}\times(J\boldsymbol{\omega})
\right]\\
&-\dot{\boldsymbol{\omega}}_{\mathrm{ref}}
-\dot{\boldsymbol{\omega}}_{\mathrm{err},d}
\end{aligned}
```

**Step 3: Substitute Equation (21).**

The compensation torque is:

```math
\boldsymbol{\tau}_1
=
\boldsymbol{\omega}\times(J\boldsymbol{\omega})
+
J\dot{\boldsymbol{\omega}}_{\mathrm{ref}}
+
J\dot{\boldsymbol{\omega}}_{\mathrm{err},d}
```

The gyroscopic terms cancel:

```math
\boldsymbol{\omega}\times(J\boldsymbol{\omega})
-
\boldsymbol{\omega}\times(J\boldsymbol{\omega})
=0
```

The reference acceleration terms cancel:

```math
J^{-1}J\dot{\boldsymbol{\omega}}_{\mathrm{ref}}
-
\dot{\boldsymbol{\omega}}_{\mathrm{ref}}
=0
```

The virtual-control derivative terms also cancel:

```math
J^{-1}J\dot{\boldsymbol{\omega}}_{\mathrm{err},d}
-
\dot{\boldsymbol{\omega}}_{\mathrm{err},d}
=0
```

Consequently, only the second torque component remains:

```math
\dot{\boldsymbol{\delta}}
=
J^{-1}\boldsymbol{\tau}_2
```

**Step 4: Substitute Equation (22).**

```math
\boldsymbol{\tau}_2
=
-k_2J\boldsymbol{\delta}^{1/3}
```

Therefore:

```math
\dot{\boldsymbol{\delta}}
=
J^{-1}
\left(
-k_2J\boldsymbol{\delta}^{1/3}
\right)
```

Since the scalar gain can be factored out and the inertia matrix cancels with its inverse:

```math
\boxed{
\dot{\boldsymbol{\delta}}
=
-k_2\boldsymbol{\delta}^{1/3}
}
```

This is Equation (23).

### What Does Equation (23) Do?

Equation (23) describes the intended final error behaviour after the torque controller compensates for the rotational dynamics.

The remaining error follows the selected fractional-power feedback law and is intended to reach zero in finite time under the ideal continuous-time assumptions.

### Why Do We Use Equation (23)?

It provides the main convergence objective for the second backstepping stage. By designing the torque to produce these error dynamics, the controller aims to make the actual angular-velocity error follow the virtual-control command.

Together, Equations (16)–(23) establish the second backstepping stage: define the residual error, derive its dynamics, compensate for the nonlinear rotational model, and select a feedback torque that drives the error toward zero.








# Numerical Example of the Quaternion-Based Backstepping Controller

This section demonstrates how the quaternion-based backstepping controller calculates attitude error, angular velocity error, backstepping error, and control torque using numerical values.

## 1. Given Parameters

Use the following inertia matrix and controller gains:

```math
J=
\begin{bmatrix}
0.1&0&0\\
0&0.1&0\\
0&0&0.12
\end{bmatrix}
\ \mathrm{kg\,m^2}
```

```math
k_1=20,\qquad k_2=2
```

For this example, assume that the actual and reference states at a particular instant are:

Actual attitude quaternion:

```math
q=
\begin{bmatrix}
0.965926\\
0\\
0\\
0.258819
\end{bmatrix}
```

Reference attitude quaternion:

```math
q_{\mathrm{ref}}=
\begin{bmatrix}
1\\
0\\
0\\
0
\end{bmatrix}
```

Actual angular velocity:

```math
\boldsymbol{\omega}=
\begin{bmatrix}
0.2\\
-0.1\\
0.3
\end{bmatrix}
\ \mathrm{rad/s}
```

Reference angular velocity:

```math
\boldsymbol{\omega}_{\mathrm{ref}}=
\begin{bmatrix}
0.1\\
-0.1\\
0.2
\end{bmatrix}
\ \mathrm{rad/s}
```

For the torque calculation, assume that the reference angular acceleration at this instant is:

```math
\dot{\boldsymbol{\omega}}_{\mathrm{ref}}=
\begin{bmatrix}
0.05\\
0\\
-0.02
\end{bmatrix}
\ \mathrm{rad/s^2}
```

## 2. Calculate the Quaternion Attitude Error

Using Equation (10):

```math
q_{\mathrm{err}}=
q_{\mathrm{ref}}^*\otimes q
```

The conjugate of the reference quaternion is:

```math
q_{\mathrm{ref}}^*=
\begin{bmatrix}
1\\
0\\
0\\
0
\end{bmatrix}
```

Since this is the identity quaternion, its multiplication leaves the actual quaternion unchanged.

Therefore:

```math
\boxed{
q_{\mathrm{err}}=
\begin{bmatrix}
0.965926\\
0\\
0\\
0.258819
\end{bmatrix}
}
```

The scalar component is:

```math
\eta=0.965926
```

The vector component is:

```math
\boldsymbol{\epsilon}=
\begin{bmatrix}
0\\
0\\
0.258819
\end{bmatrix}
```

This quaternion represents an illustrative attitude error of approximately 30 degrees about the third axis.

## 3. Calculate the Desired Angular Velocity Error

Using the vector-part virtual control from Equation (13):

```math
\boldsymbol{\omega}_{\mathrm{err},d}
=
-k_1\boldsymbol{\epsilon}
```

Substitute the numerical values:

```math
\boldsymbol{\omega}_{\mathrm{err},d}
=
-20
\begin{bmatrix}
0\\
0\\
0.258819
\end{bmatrix}
```

Therefore:

```math
\boxed{
\boldsymbol{\omega}_{\mathrm{err},d}
=
\begin{bmatrix}
0\\
0\\
-5.17638
\end{bmatrix}
\ \mathrm{rad/s}
}
```

This is the desired angular velocity error generated by the first backstepping stage.

## 4. Calculate the Angular Velocity Tracking Error

Using Equation (11):

```math
\boldsymbol{\omega}_{\mathrm{err}}
=
\boldsymbol{\omega}
-
\boldsymbol{\omega}_{\mathrm{ref}}
```

Substitute the actual and reference angular velocities:

```math
\boldsymbol{\omega}_{\mathrm{err}}
=
\begin{bmatrix}
0.2\\
-0.1\\
0.3
\end{bmatrix}
-
\begin{bmatrix}
0.1\\
-0.1\\
0.2
\end{bmatrix}
```

Subtract component by component:

```math
\boxed{
\boldsymbol{\omega}_{\mathrm{err}}
=
\begin{bmatrix}
0.1\\
0\\
0.1
\end{bmatrix}
\ \mathrm{rad/s}
}
```

The result shows that the actual angular velocity differs from the reference angular velocity along the first and third axes.

## 5. Calculate the Backstepping Error

Using Equation (16):

```math
\boldsymbol{\delta}
=
\boldsymbol{\omega}_{\mathrm{err}}
-
\boldsymbol{\omega}_{\mathrm{err},d}
```

Substitute the calculated values:

```math
\boldsymbol{\delta}
=
\begin{bmatrix}
0.1\\
0\\
0.1
\end{bmatrix}
-
\begin{bmatrix}
0\\
0\\
-5.17638
\end{bmatrix}
```

Therefore:

```math
\boxed{
\boldsymbol{\delta}
=
\begin{bmatrix}
0.1\\
0\\
5.27638
\end{bmatrix}
\ \mathrm{rad/s}
}
```

The backstepping error measures the difference between the actual angular velocity error and the desired angular velocity error.

The torque controller now attempts to drive this error toward zero.

## 6. Calculate the Gyroscopic Term

The gyroscopic term in Equation (9) is:

```math
\boldsymbol{\omega}\times(J\boldsymbol{\omega})
```

First, calculate angular momentum in body coordinates:

```math
J\boldsymbol{\omega}
=
\begin{bmatrix}
0.1&0&0\\
0&0.1&0\\
0&0&0.12
\end{bmatrix}
\begin{bmatrix}
0.2\\
-0.1\\
0.3
\end{bmatrix}
```

Therefore:

```math
J\boldsymbol{\omega}
=
\begin{bmatrix}
0.02\\
-0.01\\
0.036
\end{bmatrix}
```

Now calculate the cross product:

```math
\boldsymbol{\omega}\times(J\boldsymbol{\omega})
=
\begin{bmatrix}
-0.1(0.036)-0.3(-0.01)\\
0.3(0.02)-0.2(0.036)\\
0.2(-0.01)-(-0.1)(0.02)
\end{bmatrix}
```

Simplifying:

```math
\boxed{
\boldsymbol{\omega}\times(J\boldsymbol{\omega})
=
\begin{bmatrix}
-0.0006\\
-0.0012\\
0
\end{bmatrix}
\ \mathrm{N\,m}
}
```

This term accounts for the coupling between angular velocity and angular momentum.

## 7. Calculate the Derivative of the Virtual Control

The vector-part virtual control is:

```math
\boldsymbol{\omega}_{\mathrm{err},d}
=
-k_1\boldsymbol{\epsilon}
```

Since the gain is constant:

```math
\dot{\boldsymbol{\omega}}_{\mathrm{err},d}
=
-k_1\dot{\boldsymbol{\epsilon}}
```

Using the vector part of the quaternion error kinematics:

```math
\dot{\boldsymbol{\epsilon}}
=
\frac{1}{2}
\left(
\eta I+
[\boldsymbol{\epsilon}]_{\times}
\right)
\boldsymbol{\omega}_{\mathrm{err}}
```

where:

```math
[\boldsymbol{\epsilon}]_{\times}
=
\begin{bmatrix}
0&-\epsilon_3&\epsilon_2\\
\epsilon_3&0&-\epsilon_1\\
-\epsilon_2&\epsilon_1&0
\end{bmatrix}
```

For the current values:

```math
\eta=0.965926
```

```math
\boldsymbol{\epsilon}=
\begin{bmatrix}
0\\
0\\
0.258819
\end{bmatrix}
```

Therefore:

```math
\dot{\boldsymbol{\epsilon}}
=
\frac{1}{2}
\begin{bmatrix}
0.965926&-0.258819&0\\
0.258819&0.965926&0\\
0&0&0.965926
\end{bmatrix}
\begin{bmatrix}
0.1\\
0\\
0.1
\end{bmatrix}
```

The result is:

```math
\dot{\boldsymbol{\epsilon}}
\approx
\begin{bmatrix}
0.0482963\\
0.0129410\\
0.0482963
\end{bmatrix}
```

Multiplying by the negative gain:

```math
\dot{\boldsymbol{\omega}}_{\mathrm{err},d}
=
-20
\begin{bmatrix}
0.0482963\\
0.0129410\\
0.0482963
\end{bmatrix}
```

Therefore:

```math
\boxed{
\dot{\boldsymbol{\omega}}_{\mathrm{err},d}
\approx
\begin{bmatrix}
-0.965926\\
-0.258819\\
-0.965926
\end{bmatrix}
\ \mathrm{rad/s^2}
}
```

This term accounts for the changing virtual-control command in the torque calculation.

## 8. Calculate the Compensation Torque

Using Equation (21):

```math
\boldsymbol{\tau}_1
=
\boldsymbol{\omega}\times(J\boldsymbol{\omega})
+
J\dot{\boldsymbol{\omega}}_{\mathrm{ref}}
+
J\dot{\boldsymbol{\omega}}_{\mathrm{err},d}
```

First, calculate the reference-acceleration contribution:

```math
J\dot{\boldsymbol{\omega}}_{\mathrm{ref}}
=
\begin{bmatrix}
0.1&0&0\\
0&0.1&0\\
0&0&0.12
\end{bmatrix}
\begin{bmatrix}
0.05\\
0\\
-0.02
\end{bmatrix}
```

```math
J\dot{\boldsymbol{\omega}}_{\mathrm{ref}}
=
\begin{bmatrix}
0.005\\
0\\
-0.0024
\end{bmatrix}
```

Next, calculate the virtual-control derivative contribution:

```math
J\dot{\boldsymbol{\omega}}_{\mathrm{err},d}
\approx
\begin{bmatrix}
-0.0965926\\
-0.0258819\\
-0.1159111
\end{bmatrix}
```

Combine all three terms:

```math
\boldsymbol{\tau}_1
=
\begin{bmatrix}
-0.0006\\
-0.0012\\
0
\end{bmatrix}
+
\begin{bmatrix}
0.005\\
0\\
-0.0024
\end{bmatrix}
+
\begin{bmatrix}
-0.0965926\\
-0.0258819\\
-0.1159111
\end{bmatrix}
```

Therefore:

```math
\boxed{
\boldsymbol{\tau}_1
\approx
\begin{bmatrix}
-0.0921926\\
-0.0270819\\
-0.1183111
\end{bmatrix}
\ \mathrm{N\,m}
}
```

This component compensates for the rotational dynamics and changing reference signals.

## 9. Calculate the One-Third Power of the Backstepping Error

The second torque component uses:

```math
\boldsymbol{\tau}_2
=
-k_2J\boldsymbol{\delta}^{1/3}
```

The backstepping error is:

```math
\boldsymbol{\delta}
=
\begin{bmatrix}
0.1\\
0\\
5.27638
\end{bmatrix}
```

Calculate its real cube root component by component:

```math
\boldsymbol{\delta}^{1/3}
=
\begin{bmatrix}
\sqrt[3]{0.1}\\
\sqrt[3]{0}\\
\sqrt[3]{5.27638}
\end{bmatrix}
```

Therefore:

```math
\boxed{
\boldsymbol{\delta}^{1/3}
\approx
\begin{bmatrix}
0.464159\\
0\\
1.74092
\end{bmatrix}
}
```

The cube root preserves the sign of negative error components as well. This is important because the feedback must drive errors toward zero regardless of their signs.

## 10. Calculate the Feedback Torque

Using Equation (22):

```math
\boldsymbol{\tau}_2
=
-k_2J\boldsymbol{\delta}^{1/3}
```

Substitute the numerical values:

```math
\boldsymbol{\tau}_2
=
-2
\begin{bmatrix}
0.1&0&0\\
0&0.1&0\\
0&0&0.12
\end{bmatrix}
\begin{bmatrix}
0.464159\\
0\\
1.74092
\end{bmatrix}
```

First multiply by the inertia matrix:

```math
J\boldsymbol{\delta}^{1/3}
\approx
\begin{bmatrix}
0.0464159\\
0\\
0.2089104
\end{bmatrix}
```

Then multiply by the negative gain:

```math
\boxed{
\boldsymbol{\tau}_2
\approx
\begin{bmatrix}
-0.0928318\\
0\\
-0.4178208
\end{bmatrix}
\ \mathrm{N\,m}
}
```

This is the torque component that drives the backstepping error toward zero under the intended error dynamics.

## 11. Calculate the Final Control Torque

The total torque is the sum of the compensation torque and the feedback torque:

```math
\boldsymbol{\tau}
=
\boldsymbol{\tau}_1+\boldsymbol{\tau}_2
```

Substituting the calculated values:

```math
\boldsymbol{\tau}
\approx
\begin{bmatrix}
-0.0921926\\
-0.0270819\\
-0.1183111
\end{bmatrix}
+
\begin{bmatrix}
-0.0928318\\
0\\
-0.4178208
\end{bmatrix}
```

Therefore, the final control torque is approximately:

```math
\boxed{
\boldsymbol{\tau}
\approx
\begin{bmatrix}
-0.1850244\\
-0.0270819\\
-0.5361319
\end{bmatrix}
\ \mathrm{N\,m}
}
```

These values are the calculated control torques for the illustrative state and derivative values used in this example.

## 12. Calculate the Backstepping Error Derivative

Using Equation (23):

```math
\dot{\boldsymbol{\delta}}
=
-k_2\boldsymbol{\delta}^{1/3}
```

Substitute the calculated cube root:

```math
\dot{\boldsymbol{\delta}}
=
-2
\begin{bmatrix}
0.464159\\
0\\
1.74092
\end{bmatrix}
```

Therefore:

```math
\boxed{
\dot{\boldsymbol{\delta}}
\approx
\begin{bmatrix}
-0.928318\\
0\\
-3.48184
\end{bmatrix}
}
```

The first and third error components have negative derivatives because their current values are positive. Both components therefore move toward zero under the designed error dynamics.

The second component is already zero and remains at zero in the ideal model.

## 13. Estimate the Ideal Convergence Time

For each scalar error component, the ideal time to reach zero under the one-third-power feedback law is:

```math
T_i=
\frac{3}{2k_2}
|\delta_i(0)|^{2/3}
```

Using the selected gain:

```math
k_2=2
```

For the first component:

```math
T_1=
\frac{3}{4}(0.1)^{2/3}
```

```math
\boxed{
T_1\approx0.1616\ \mathrm{s}
}
```

For the second component, the initial error is zero:

```math
\boxed{
T_2=0
}
```

For the third component:

```math
T_3=
\frac{3}{4}(5.27638)^{2/3}
```

```math
\boxed{
T_3\approx2.2731\ \mathrm{s}
}
```

These are ideal scalar-model estimates for the chosen initial errors. They are not measured simulation results and should not be confused with the convergence times reported in the research paper.

## 14. Numerical Example Summary

The calculations demonstrate the complete sequence of the backstepping controller:

1. The quaternion error measures the attitude mismatch.
2. The virtual control calculates the desired angular velocity correction.
3. The angular velocity error measures the difference between actual and reference motion.
4. The backstepping error measures the difference between the actual error and the virtual-control command.
5. The compensation torque accounts for rotational coupling and changing reference signals.
6. The fractional-power feedback torque drives the backstepping error toward zero.

The final torque is obtained by combining the compensation and feedback components.

The example illustrates how the quaternion-based backstepping controller converts attitude and angular velocity errors into control torque. Actual convergence and tracking performance must still be evaluated using the complete simulation model, including numerical integration, disturbances, and actuator constraints.
