import numpy as np
import matplotlib.pyplot as plt 

# arbitrary system constants
MASS = 5   
SPRING = 10
DAMPEN = 7

# KF class
class KalmanFilter:

    def __init__(self):
        return
    
# representing mass-spring system
class DynamicSystem:

    def __init__(self, A:np.ndarray,
                       B:np.ndarray,
                       C:np.ndarray,
                       D:np.ndarray,
                       Q:np.ndarray,
                       R:np.ndarray,
                       L:np.ndarray,
                       M:np.ndarray,
                       x0:np.ndarray):

        """
        setup LTI system:
        xDot = Ax + Bu + Lw
        y = Cx + Du + Mv
        """
        
        self.A = A
        self.B = B
        self.L = L
        self.Q = Q
        
        self.C = C
        self.D = D
        self.M = M
        self.R = R
        

        self.ideal_state:np.ndarray = x0    # no noise
        self.real_state:np.ndarray = x0 # input noise
        self.meas_state:np.ndarray = x0     # input and obs noise

        return
    
    def propagate(self, delT:float, input:np.ndarray)->np.ndarray:

        self.ideal_state = self.__update_state(delT, self.ideal_state, input, noise=np.zeros(self.Q.shape))
        self.real_state = self.__update_state(delT, self.real_state, input, noise=self.Q)
        self.meas_state = self.__measure_state(input)
    
        return self.meas_state
    
    
    def __update_state(self, delT:float, state:np.ndarray, input_control:np.ndarray, noise:np.ndarray)->np.ndarray:

        W = np.array([np.random.normal(covar) for covar in noise])
        
        del_state = self.A @ state + self.B @ input_control + self.L @ W

        new_state = state + del_state*delT

        return new_state
    
    def __measure_state(self, input_control:np.ndarray)->np.ndarray:

        V = np.array([np.random.normal(covar) for covar in self.R])
        obs_state = self.C @ self.real_state + self.D @ input_control + self.M @ V

        return obs_state

def plot_results(time:np.ndarray, full_data_set:np.ndarray[np.ndarray[float]], labels:np.ndarray[str],
                 *, num_rows:int=None, num_cols:int=1)->None:
    """
    Simple utility function to plot data points

    Args:
        time (np.ndarray): time array
        full_data_set (np.ndarray[np.ndarray[float]]): array of datasets 
        labels (np.ndarray[str]): list of labels in order
        num_rows (int, optional): number of rows in figure. Defaults to None.
        num_cols (int, optional): number of cols in figure. Defaults to 1.
    """
    if num_rows is None:
        num_rows = full_data_set.shape[0]

    fig = plt.figure()
    
    for i in range(num_rows):
        for j in range(num_cols):
            idx = num_cols * i + j
            ax = fig.add_subplot(num_rows, num_cols, idx + 1)
            ax.plot(time, full_data_set[idx], label=labels[idx])
            ax.legend()

    plt.show()

    return

def simulate():
    
    # arbitrary timing
    time = np.linspace(0, 10, 10001)
    input = np.sin(2*np.pi * time / 2.5)    # arbitrary input

    x0 = np.array([5, 0])
    cart_A = np.array([[0, 1], [-SPRING/MASS, -DAMPEN/MASS]])
    cart_B = np.array([0, 1/MASS])   # see README for derivation
    cart_L = np.eye(len(x0))
    Q = np.array([0.001, 0.001])  # accurate control on input

    cart_C = np.eye(len(x0))    # assume full observability
    cart_D = np.zeros(len(x0))         # assume no observation input
    cart_M = np.eye(len(x0))
    R = np.array([0.001, .01])   # relatively noise data

    cart_system = DynamicSystem(cart_A, cart_B, cart_C, cart_D, Q, R, cart_L, cart_M, x0)

    ideal_history = np.zeros((2, len(time))).T
    real_history = np.zeros((2, len(time))).T
    meas_history = np.zeros((2, len(time))).T

    ideal_history[0] = cart_system.ideal_state
    real_history[0] = cart_system.real_state
    meas_history[0] = cart_system.meas_state

    for i in range(1, len(time)):
        delt = time[i] - time[i-1]
        cart_system.propagate(delt, np.array([0,input[i]]))

        ideal_history[i] = cart_system.ideal_state
        real_history[i] = cart_system.real_state
        meas_history[i] = cart_system.meas_state

    fig = plt.figure()
    ax = fig.add_subplot(311)
    ax.plot(time, ideal_history.T[0], label='Pos')
    ax.plot(time, ideal_history.T[1], label='Vel')
    ax.set_title('Ideal')
    ax.legend()

    ax = fig.add_subplot(312)
    ax.plot(time, real_history.T[0], label='Pos')
    ax.plot(time, real_history.T[1], label='Vel')
    ax.set_title('Real')
    ax.legend()
    
    ax = fig.add_subplot(313)
    ax.plot(time, meas_history.T[0], label='Pos')
    ax.plot(time, meas_history.T[1], label='Vel')
    ax.set_title('Measured')
    ax.legend()

    plt.show()

    return

if __name__ == '__main__':
    simulate()