import numpy as np
import matplotlib.pyplot as plt


class GenerateTrajectory():
    def __init__(self):
        self.t = np.arange(-1,1,0.004)

        self.a_x = self.a_long(self.t)
        self.a_y = self.a_lat(self.t)
        self.a_z = self.a_vert(self.t)

        self.v_x = np.clip(2*self.simpson_integration(0.5,self.a_x,0.1)-3,-10,10)
        self.v_y = np.clip(0.5*self.simpson_integration(2,self.a_y,0.1),-10,10)
        self.v_z = np.clip(self.simpson_integration(-2,self.a_z,0.1),-2,2)

        self.p_x = self.simpson_integration(0,self.v_x,0.1)
        self.p_y = self.simpson_integration(0,self.v_y,0.1)
        self.p_z = np.clip(self.simpson_integration(20,self.v_z,0.1),5,35)

        
    def a_long(self,x):
        p = 9*x**4 - 5*x + 5
        return 2*(np.tanh(p)+np.cos(12*x**2-2))/2  -1

    def a_lat(self,x):
        p = 10*x**4 -3*x**3- 2*x + -2

        return np.clip(2*((np.tanh(p)  +0.8*np.sin(8*x**5))/2),-5,5)+0


    def a_vert(self,x):
        p = 1.5*x**6 - 2*x + 5

        return 2*((np.tanh(p/1)  +0.8*np.sin(3*x**3)/np.cos(x))/2)

    def simpson_integration(self,y0, y, h):
        y = np.asarray(y)
        n = y.size
        S = np.zeros(n, dtype=float)
        if n == 0:
            return S
        S[0] = y0
        if n == 1:
            return S

        # First step: trapezoid
        S[1] = S[0] + 0.5*h*(y[0] + y[1])

        # Advance in blocks of two with Simpson; fill odd indices by trapezoid from the last even
        for k in range(2, n, 2):
            S[k] = S[k-2] + (h/3.0)*(y[k-2] + 4.0*y[k-1] + y[k])
            if k + 1 < n:
                S[k+1] = S[k] + 0.5*h*(y[k] + y[k+1])

        return S


    def plot(self):


        fig, axes = plt.subplots(3,1, figsize=(10, 10))
        t_plot = (50/2)*(self.t+1)
        print(t_plot)
        axes[0].plot(t_plot, self.a_x,label = "$a_x$")
        axes[0].plot(t_plot, self.a_y,label = "$a_y$")
        axes[0].plot(t_plot, self.a_z,label = "$a_z$")
        axes[0].legend(loc="upper left")
        axes[0].grid()
        axes[1].plot(t_plot, self.v_x,label = "$v_x$")
        axes[1].plot(t_plot, self.v_y,label = "$v_y$")
        axes[1].plot(t_plot, self.v_z,label = "$v_z$")
        axes[1].legend()
        axes[1].grid()
        axes[2].plot(t_plot, self.p_x,label = "$p_x$")
        axes[2].plot(t_plot, self.p_y,label = "$p_y$")
        axes[2].plot(t_plot, self.p_z,label = "$p_z$")
        axes[2].legend()
        axes[2].grid()
        axes[0].set_title("Realistic Trajectory")
        # axes[1].set_title("Velocity")
        # axes[2].set_title("Position")
        # axes[0].set_xlabel("Time [s]")
        # axes[1].set_xlabel("Time [s]")
        axes[2].set_xlabel("t [s]")
        axes[0].set_ylabel("Acceleration $[m/s^2]$")
        axes[1].set_ylabel("Velocity $[m/s]$")
        axes[2].set_ylabel("Position $[m]$")

        plt.tight_layout()
        plt.savefig("trajectory.pdf", format="pdf", bbox_inches="tight")
        plt.show()


if __name__ == '__main__':
    #Init node
    gt = GenerateTrajectory()
    gt.plot()
