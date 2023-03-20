# **Kalman Filter and Plotting**
## **Context**
Exploration of Kalman Filters in Python and C++. Part of a larger project to simulate rocket landings in real-time in C++. This repository will hold some more theory/background on the Kalman Filter specifically as it relates to data handling and real-time data smoothing.

## **Overview**
### Dynamic System
This system analyzes a standard mass-spring-dampening system in the LTI form $\dot{x} = Ax + Bu$ and $y = Cx + Du$. The dynamic system strictly deals with the "ideal" world and does not bake in any noise or estimation. The input, however, may not be optimal is it is directly a function of the *state estimation* by the **Kalman Filter**.


### Kalman Filters
A Kalman Filter is technically a misnomer. Kalman Filters do not filter data, necessarily, but are an extremely popular state estimator using a Predictor-Corrector setup; that is, the Kalman Filter expects to know some information about the system dynamics which it will use to predict the next state (given the input to the system). It will then correct its prediction given the next measurement and update the **Covariance Matrix**. This, essentially, is represents the variance of the states with themselves and the other states and, essentially, is good the knowledge of the data is. If the data is very "clean" or is well known then the covariance will be low. However, if the data is noisy or moves too quickly then the covariance will be high; the system will be more uncertain of where the true data lies.

The Kalman filter is ideal based on the calculation of the Kalman Gain, $K$. The Kalman Gain attempts to minimize the covariance of the state via an Optimal Control approach; e.g., 

$$
J(\mathbf{K}) = \text{tr}[\mathbf{P}]
$$

$$
\text{(Spacecraft Dynamics and Control; DeRuiter)}
$$

where $J$ is the cost function being minimized and the "tr" function is the trace whereby the main diagonal of $\mathbf{P}$ is the covariance of the state. 


## **The System**
For arbitrary spring constant, $k$, dampening coefficient, $c$, mass, $m$, and input force, $F$, the system dynamics can be written as 

$$
m\ddot{x} = F - c\dot{x} - kx \\
$$

$$
\ddot{x} = \frac{F - c\dot{x} - kx}{m}
$$

If the state vector, $\bar{x}$ is defined as the position and velocity and $u$ is defined as the input force $F$, the state derivative can be derived using (2).

$$
\bar{x} = \begin{bmatrix}
x \\
\dot{x}
\end{bmatrix} =  \begin{bmatrix}
s_1 \\
s_2
\end{bmatrix}
$$

$$
\bar{u} = \begin{bmatrix} F \end{bmatrix} = \begin{bmatrix} u_1 \end{bmatrix}
$$

$$
\dot{\bar{x}} = \begin{bmatrix}
\dot{x} \\
\ddot{x}
\end{bmatrix} = \begin{bmatrix}
\dot{x} \\
\frac{F - c\dot{x} - kx}{m}
\end{bmatrix} = \begin{bmatrix}
s_2 \\
\frac{1}{m}\left(u_1 - cs_2 - ks_1\right)
\end{bmatrix}
$$

Looking at the formulation for $\dot{\bar{x}}$, the state matrices $A, B, C, D$ need to be determined in terms of $\bar{x}$. In fear of being too verbose, $A, B, C,$ and $D$ are shown below. Multiplying the matrices by the correct factor will prove the formulations hold true with the form $\dot{x} = Ax + Bu$ and $y = Cx + Du$.

$$
A = \begin{bmatrix}
0 & 1 \\
-\frac{k}{m} & -\frac{c}{m}
\end{bmatrix}
$$

$$
B = \begin{bmatrix}
0 \\
\frac{1}{m}
\end{bmatrix}
$$

$$
C = \begin{bmatrix}
1 & 0 \\
0 & 1
\end{bmatrix}
$$

$$
D = \begin{bmatrix}
0 \\
0
\end{bmatrix}
$$

Notice: the system has full observability as shown by $C$, that is, the system can directly measure the full state. This may not be the case for some systems and instead must rely an indirect measurement form (e.g., resistance as a form to measure heat).

While this system can be solved analytically using the [state transformation](https://web.mit.edu/2.14/www/Handouts/StateSpaceResponse.pdf), another method is to propagate the system manually (e.g., with a Runge Kutta solver) and keep track of the states and inputs. This is what this mini-project will do to explore Kalman Filtering as it requires the system dynamics be known and used during the prediction step.

## **The Kalman Filter**
*Spacecraft Dynamics and Control: An Introduction* by DeRuiter (2013) provides a fantastic introduction to the derivation and process of Kalman Filtering. A snippet of the Kalman Filter chapter is outlined below. Some variable explanations first.

### System Dynamics
$$
x_k = F_{k-1}x_{k-1} + G_{k-1}u_{k-1} + L_{k-1}w_{k-1} \\
$$

$$
y_k = H_kx_k + M_kv_k \\
$$

$$
w_k \sim \sigma(0, Q_k) \\
$$

$$
v_k \sim \sigma(0, R_k) \\
$$

Where the subscript represents the time index, $x_k$ represents the state estimation, $y_k$ represents the state estimate measurement, $F_i$ represents the dynamics of the system at time $i$, $G_i$ represents the affect of the input on the state at time $i$, $L$ and $M$ represent the mapping of the noise to the state and measurement respectively, $w$ and $v$ represent the noise in the state estimation and measurement process respectively, and $Q$ and $R$ represent the covariance of the estimation and measurement processes respectively. For time-invariant system (e.g., the system dynamics do not change over time), $F, G, L, H, M$ can all be held constant instead of being evaluated at each timestep.

The Predictor-Corrector setup is also given by DeRuiter and is summarized below

### Prediction

$$
\hat{x}^-_k = F\hat{x}_{k-1} + Gu_{k-1} \\
$$

$$
P_k^- = FP_{k-1}F^T + LQL^T \\
$$

$$
\text{(Spacecraft Dynamics and Control; DeRuiter)}
$$

Where the minus subscript, $\square^-$, represents the *a priori*, or "before knowledge" estimate. This is the current best estimate of the state *before* any new measurements are added. This can be used alone if there is a blackout in data-acquisition, however, errors may accumulate over time.

### Correction

$$
W_k = HP_k^-H^T + MRM^T \\
$$

$$
K_k = P_k^-H^TW^{-1}_k \\
$$

$$
\hat{x}_k = \hat{x}^-_k + K_k(y_k - \hat{y}_k^-) \\
$$

$$
P_k = P_k^- - K_kHP_k^- - P_k^-H^TK_k^T + K_kW_kK_k^T
$$

Notice that the current best estimate of the state, $\hat{x}_k$, has been updated based on the new measurement data where $y_k$ is the true measurement recorded and $\hat{y}_k^-$ is the estimate of the measurement based on the *a priori* estimate; e.g.,

$$
\hat{y}_k^- = H_k\hat{x}_k^- + M_kv_k
$$

The covariance of the system is also updated to reflect the confidence in the estimate.

## **Misc**
### Covariance Matrix
The Covariance Matrix, $P$, is an $m \text{ x } m$ matrix where $m$ is the number of states in the system; e.g., 

$$
x \in \mathcal{R}^{m}
$$

$P$ encodes the following information:

$$
P = \begin{bmatrix}
\sigma_1^2 & \sigma_1\sigma_2 & \dots & \sigma_1\sigma_m \\
\sigma_1\sigma_2 & \sigma_2^2 & \dots & \sigma_2\sigma_m \\
\vdots & \vdots & \ddots & \vdots \\
\sigma_m\sigma_1 & \sigma_m\sigma_2 & \dots & \sigma_m^2
\end{bmatrix}
$$

where an element of $P$, $P_{ij}$, represents the covariance of state $i$ and state $j$. **Aside:** notice that $P$ is symmetric as $\sigma_1\sigma_2$ is equivalent to $\sigma_2\sigma_1$; this can help improve computation in some cases by exploiting the properties of symmetric matrices. If $i == j$ then it is said that $P_{ij}$ represents the variance of that state. In practice, the variance of the state represents the confidence of that state; a higher variance value correlates with lower confidence and visa versa. The goal for the Kalman Gain, $K$, is to minimize the main diagonal of $P$, that is, to minimize the variance of the states (and thereby maximizing confidence in the estimate).