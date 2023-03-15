# KalmanFilter
Exploration of Kalman Filters in Python and C++. Part of a larger project to simulate rocket landings in faster-than-real-time in C++. This repository will hold some more theory/background on the Kalman Filter specifically as it relates to data handling and real-time data smoothing. This system analyzes a standard mass-spring-dampening system in the LTI form $\dot{x} = Ax + Bu$.

## The System
For arbitrary spring constant, $k$, dampening coefficient, $c$, mass, $m$, and input force, $F$, the system dynamics can be written as 

$\begin{align}
m\ddot{x} = F - c\dot{x} - kx \\
\end{align}$

$\begin{align}
\ddot{x} = \frac{F - c\dot{x} - kx}{m}
\end{align}$

If the state vector, $\bar{x}$ is defined as the position and velocity and $u$ is defined as the input force $F$, the state derivative can be derived using (2).

$\begin{align}
\bar{x} = \begin{bmatrix}
x \\ \dot{x}
\end{bmatrix} =  \begin{bmatrix}
s_1 \\ s_2
\end{bmatrix}
\quad u =  \begin{bmatrix}
0 \\ F
\end{bmatrix} = \begin{bmatrix}
u_1 \\ u_2
\end{bmatrix}\\
\dot{\bar{x}} = \begin{bmatrix} \dot{x} \\ \ddot{x} \end{bmatrix} = \begin{bmatrix}
\dot{x} \\
\frac{F - c\dot{x} - kx}{m}
\end{bmatrix} = \begin{bmatrix}
s_2 \\
\frac{1}{m}\left(u_2 - cs_2 - ks_1\right)
\end{bmatrix}
\end{align}$

While this can be solved analytically using the [state transformation](https://web.mit.edu/2.14/www/Handouts/StateSpaceResponse.pdf), another method is to propagate the system manually (e.g., with a Runge Kutta solver) and keep track of the states and inputs. This is what this mini-project will do to explore Kalman Filtering as it requires the system dynamics be known and used during the prediction step.

## Kalman Filters
Kalman Filters are a popular filter type for handling noisy data using a Predictor-Corrector setup; that is, the Kalman Filter expects to know some information about the system dynamics which it will use to predict the next state (given the input to the system). It will then correct its prediction given the next measurement and update the **Covariance Matrix**. This, essentially, is represents the variance of the states with themselves and the other states and, essentially, is good the knowledge of the data is. If the data is very "clean" or is well known then the covariance will be low. However, if the data is noisy or moves too quickly then the covariance will be high; the system will be more uncertain of where the true data lies.

<!-- ## Application -->