import numpy as np
import matplotlib.pyplot as plt 
from estimators import *

# arbitrary system constants
MASS = 3   
SPRING = 20
DAMPEN = 2

# representing mass-spring system
class DynamicSystem:

    def __init__(self, A:np.ndarray,
                       B:np.ndarray,
                       C:np.ndarray,
                       D:np.ndarray,
                       x0:np.ndarray,
                       estimator:StateEstimator=StateEstimator()):

        """
        setup LTI system:
        xDot = Ax + Bu
        y = Cx + Du
        """
        
        self.A = A
        self.B = B
        
        self.C = C
        self.D = D
        
        self.est = estimator

        self.ideal_state:np.ndarray = x0    # no noise
        self.real_state:np.ndarray = x0 # input noise
        self.meas_state:np.ndarray = x0     # input and obs noise

        return
    
    def propagate(self, delT:float, input:np.ndarray)->np.ndarray:

        self.ideal_state = self.__update_state(delT, self.ideal_state, input, noise=np.zeros(self.Q.shape))
        self.real_state = self.__update_state(delT, self.real_state, input, noise=self.Q)
        self.meas_state = self.__measure_state(input)
    
        return self.meas_state


def system_init()->DynamicSystem:
    """
    Hides System Initialization behind function for cleanliness of main

    Returns:
        DynamicSystem: LTI System with arbitrary dynamics
    """
    return

def simulate():
    """
    Simulation handler
    """


    # arbitrary timing
    time = np.linspace(0, 10, 10001)
    f_input = np.sin(2*np.pi * time / 2.5)    # arbitrary forcing function

    return

if __name__ == '__main__':
    simulate()