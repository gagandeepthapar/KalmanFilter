import numpy as np
import matplotlib.pyplot as plt 

class ExtendedKalmanFilter:

    def __init__(self, x0:np.ndarray=None, P0:np.ndarray=None):

        self.xHat = x0
        self.P = P0

        return
    
    def add_measurement(self, measurement:np.ndarray)->np.ndarray:



        return self.xHat
    

def plot_results(time:np.ndarray, true:np.ndarray, measured:np.ndarray, filtered:np.ndarray)->None:

    num_states = true.shape[0]

    fig = plt.figure()
    
    for idx in range(num_states):
        ax = fig.add_subplot(num_states, 2, 2*idx + 1)

        ax.plot(time, true[idx], c='g', label='True')
        ax.plot(time, measured[idx], c='b', label='Measured from Sensor', alpha=0.3)
        ax.plot(time, filtered[idx], c='k', linestyle='dashdot', label='Kalman Filtered')
        ax.legend()
        ax.set_title('State {}'.format(idx+1))
        
        ax = fig.add_subplot(num_states, 2, 2*idx+2)
        ax.plot(time, np.zeros(time.shape), c='r', linestyle='--', label='True')
        ax.plot(time, true[idx] - measured[idx], c='b', label='Error in Measurement', alpha=0.3)
        ax.plot(time, true[idx] - filtered[idx], c='k', linestyle='dashdot', label='Error in Filter')
        ax.legend()
        ax.set_title('State {}: Error'.format(idx+1))

        

    plt.show()

    return

def simulate():

    covarA = 0.05
    covarB = 0.2
    covarC = 0.1

    # arbitrary timing
    time = np.linspace(0, 100, 1001)
    
    trueA = np.sin(2*np.pi*time / 25)
    trueB = np.cos(2*np.pi*time / 50)
    trueC = np.exp(-time * 0.1)
    
    measA = trueA + np.random.normal(0, covarA, trueA.shape)
    measB = trueB + np.random.normal(0, covarB, trueB.shape)
    measC = trueC + np.random.normal(0, covarC, trueC.shape)

    true = np.vstack([trueA, trueB, trueC])
    meas = np.vstack([measA, measB, measC])
    filt = np.zeros(true.shape)

    kf = ExtendedKalmanFilter()
    plot_results(time, true, meas, filt)

    return

if __name__ == '__main__':
    simulate()