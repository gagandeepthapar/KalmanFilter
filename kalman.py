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

    def __init__(self):
        return

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

def state_transition(del_time:float, state:np.ndarray)->np.ndarray:

    return

def simulate():
    
    # arbitrary timing
    time = np.linspace(0, 10, 10001)
    
    f = 0.3*np.sin(2*np.pi*time/2.5)    # input force, u

    return

if __name__ == '__main__':
    simulate()