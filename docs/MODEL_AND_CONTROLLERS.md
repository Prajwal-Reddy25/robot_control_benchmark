# Model and controller derivations

## Differential-drive and unicycle model

Let wheel radius be \(r\), wheel-center separation be \(b\), and left/right wheel angular rates
be \(\dot\phi_L,\dot\phi_R\). With pure rolling and no lateral velocity,

\[
v = \frac{r}{2}(\dot\phi_R+\dot\phi_L), \qquad
\omega = \frac{r}{b}(\dot\phi_R-\dot\phi_L).
\]

The pose \({\bf x}=[x,y,\theta]^T\) follows the unicycle kinematics

\[
\dot x=v\cos\theta,\qquad \dot y=v\sin\theta,\qquad \dot\theta=\omega.
\]

For constant command over \(\Delta t\), the simulator uses the exact zero-order-hold update

\[
\begin{aligned}
x_{k+1}&=x_k+\frac{v}{\omega}
  [\sin(\theta_k+\omega\Delta t)-\sin\theta_k],\\
y_{k+1}&=y_k-\frac{v}{\omega}
  [\cos(\theta_k+\omega\Delta t)-\cos\theta_k],\\
\theta_{k+1}&=\operatorname{wrap}(\theta_k+\omega\Delta t),
\end{aligned}
\]

with its continuous \(\omega\to0\) straight-line limit. Python and C++ implementations are tested
against straight and circular closed-form solutions.

For LQR and MPC, forward-Euler linearization about reference
\((\theta_r,v_r,\omega_r)\), with global error \(\delta{\bf x}={\bf x}-{\bf x}_r\), is

\[
A_k=\begin{bmatrix}
1&0&-\Delta t v_r\sin\theta_r\\
0&1& \Delta t v_r\cos\theta_r\\
0&0&1
\end{bmatrix},\quad
B_k=\begin{bmatrix}
\Delta t\cos\theta_r&0\\
\Delta t\sin\theta_r&0\\
0&\Delta t
\end{bmatrix}.
\]

Yaw errors are always wrapped to \([-\pi,\pi)\).

## Error definition and reference association

The reference-frame errors are

\[
\begin{bmatrix}e_\parallel\\e_\perp\end{bmatrix}=
\begin{bmatrix}\cos\theta_r&\sin\theta_r\\-\sin\theta_r&\cos\theta_r\end{bmatrix}
\begin{bmatrix}x-x_r\\y-y_r\end{bmatrix},\qquad
e_\theta=\operatorname{wrap}(\theta-\theta_r).
\]

Thus positive cross-track error is left of the reference tangent. Reference association searches a
bounded forward window and is monotonic. This prevents a geometric nearest-neighbor query from
jumping across a self-intersection or from the start to the end of a closed path.

## PID tracker

The PID-labelled controller combines proportional longitudinal and lateral correction with PID
heading feedback and reference feed-forward:

\[
v_c=v_r\cos e_\theta-k_\parallel e_\parallel,
\]

\[
\omega_c=\omega_r-k_\perp\max(|v_r|,v_{min})e_\perp
-k_p e_\theta-k_i\int e_\theta dt-k_d\dot e_\theta.
\]

The integral has symmetric anti-windup bounds. This is a practical path-tracking PID structure,
not a proof of global stability. Differentiating noisy heading makes it noise-sensitive; the common
actuator acceleration limit partially filters the command but is not a substitute for estimation.

## Pure Pursuit

A target is selected at arc distance \(L_d\) ahead. If \(\alpha\) is the target bearing in the
robot frame and \(d\) its Euclidean distance, the commanded curvature is

\[
\kappa_c=\frac{2\sin\alpha}{d},\qquad \omega_c=v_c\kappa_c+k_\theta(\theta_r-\theta).
\]

Speed is reduced for large bearing error. Pure Pursuit is geometric, inexpensive, and intuitive,
but a fixed look-ahead trades corner cutting against oscillation and does not explicitly minimize a
cost or enforce constraints.

## Time-varying LQR

At every sample, the discrete algebraic Riccati equation is solved for the current \(A_k,B_k\):

\[
P=A^TPA-A^TPB(R+B^TPB)^{-1}B^TPA+Q,
\]

\[
K=(R+B^TPB)^{-1}B^TPA,\qquad {\bf u}={\bf u}_r-K\delta{\bf x}.
\]

The DARE treatment freezes the local linearization over the infinite-horizon approximation. The
external plant limiter enforces constraints, so the derived unconstrained feedback is no longer
strictly optimal while saturated. Linearization is weakest for large pose errors and near loss of
forward motion.

## Linear MPC

MPC freezes the local \(A_k,B_k\) over a configurable horizon \(N\), builds the condensed prediction
\({\bf X}=S_x\delta{\bf x}_k+S_u\Delta{\bf U}\), and minimizes

\[
J=\sum_{i=1}^{N-1}\delta x_i^TQ\delta x_i+
\delta x_N^TQ_f\delta x_N+
\sum_{i=0}^{N-1}\delta u_i^TR\delta u_i
\]

subject to box bounds on absolute \(v_r+\delta v_i\) and
\(\omega_r+\delta\omega_i\). The convex quadratic is solved by fixed-iteration projected gradient
with warm starting; the largest Hessian eigenvalue sets a valid constant step. Fixed iteration count
and no branching on convergence support repeatability. Plant acceleration limits are applied after
the optimizer and are not currently included in the prediction model.

## Disturbance and actuator model

- Measurement noise: independent zero-mean Gaussian noise on \(x,y,\theta\), used only by feedback.
- Wheel slip: mean plus independent Gaussian realization for each wheel, clipped to a physically
  bounded scale, then mapped back through differential-drive kinematics.
- Initial error: deterministic \(x,y,\theta\) offsets from the first reference sample.
- Saturation: symmetric absolute linear/angular limits followed by per-sample slew limits.

Random generators are recreated from the configured integer seed for each trial. All controllers in
a scenario therefore see statistically identical, though feedback-dependent, indexed random draws.

## Assumptions and limitations

- The offline plant is kinematic: it omits motor inductance, wheel inertia, backlash, compliance,
  terrain contact, latency, and state-estimator dynamics.
- Gaussian pose noise is a controlled stressor, not a sensor-specific perception model.
- Slip is an actuation loss model; it does not reproduce lateral skidding or contact dynamics.
- Reference association is path-based rather than strict time tracking, so reported error evaluates
  geometric tracking. This avoids penalizing benign along-path lag but is unsuitable for tasks with
  hard arrival times.
- Default gains are reasonable baselines, not exhaustively tuned optima. A fair controller claim
  would require a declared tuning budget, repeated seeds, uncertainty intervals, and hardware data.
- Gazebo validates integration and more realistic contact behavior, but the deterministic offline
  evidence must not be presented as hardware performance.

