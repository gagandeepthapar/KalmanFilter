# Kalman Filter
Exploration of Kalman Filters in Python and C++. Part of a larger project to simulate rocket landings in faster-than-real-time in C++. This repository will hold some more theory/background on the Kalman Filter specifically as it relates to data handling and real-time data smoothing. This system analyzes a standard mass-spring-dampening system in the LTI form $\dot{x} = Ax + Bu + Lw$ and $y = Cx + Du + Mv$. Here, $w$ ~ $\aleph(0, Q)$ and $v$ ~ $\aleph(0, R)$ where $Q$ is the covariance of the input state (noise in input) and $R$ is the covariance of the observation method (noise in measurement).

## The System
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
L = \begin{bmatrix}
1 & 0 \\
0 & 1
\end{bmatrix}
$$

$$
C = \begin{bmatrix}
1 & 0 \\
0 & 1
\end{bmatrix}
$$

$$
D = \begin{bmatrix} 0 \end{bmatrix}
$$

$$
M = \begin{bmatrix}
1 & 0 \\
0 & 1
\end{bmatrix}
$$

Notice: the system has full observability as shown by $C$, that is, the system can directly measure the full state, albeit, with some noise as modeled by $v$.


While this can be solved analytically using the [state transformation](https://web.mit.edu/2.14/www/Handouts/StateSpaceResponse.pdf), another method is to propagate the system manually (e.g., with a Runge Kutta solver) and keep track of the states and inputs. This is what this mini-project will do to explore Kalman Filtering as it requires the system dynamics be known and used during the prediction step.

## Kalman Filters
Kalman Filters are a popular filter type for handling noisy data using a Predictor-Corrector setup; that is, the Kalman Filter expects to know some information about the system dynamics which it will use to predict the next state (given the input to the system). It will then correct its prediction given the next measurement and update the **Covariance Matrix**. This, essentially, is represents the variance of the states with themselves and the other states and, essentially, is good the knowledge of the data is. If the data is very "clean" or is well known then the covariance will be low. However, if the data is noisy or moves too quickly then the covariance will be high; the system will be more uncertain of where the true data lies.

<!-- ## Application -->