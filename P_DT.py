import numpy as np
import Model
import pathgenerator
import scipy.signal as signal
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from alive_progress import alive_bar; import time
import matplotlib
import control

#-------------------------------Generate Path----------------------------------
pg1 = pathgenerator.PathGenerator()
pg1.max_s = 10
pg1.max_j = 100
pg1.max_a = 100
pg1.max_v = 100
pg1.calculate_path(startpos=0.0, stroke=0.05, stroke_time = 0.707 ,starttime=0.1, fraction_s = 0.0, fraction_j= 1.0, fraction_a=1.0, fraction_v=1.0)

ref = []
t = np.arange(0.0, 2.0, 0.001)
for time_instant in t:
    ref.append(pg1.generate_x(time_instant))


#----------------------------------INIT----------------------------------------------------
Resolution = 30
wc = 62.8
m = 1
#Do = False         #{True, False} 
#Controller = 'PID' #{PD, PDF, PID, PIDF}
with alive_bar(4*3*Resolution**3) as bar:
    Do = False
    for Controller in ['PD', 'PDF', 'PID', 'PIDF']:
        
        CL_Model = Model.DT_System(Controller,Do,m,wc)

        S0=np.zeros((4,Resolution,Resolution,3*Resolution))
        S = S0.copy()
        for k in range(0,Resolution):
            Ts = 2*np.pi/(2*wc * 1/(k/Resolution + 0.01) )
            ref = []
            t = np.arange(0.0, 2.0, Ts)
            for time_instant in t:
                ref.append(pg1.generate_x(time_instant))

            for i in range(0,3*Resolution):
                a = i/Resolution - 1
                for j in range(0,Resolution):
                    y = j/Resolution + 0.01
                    CL_Model.Update(wc,Ts, a1=a, y=y)
                    Num, Den = CL_Model.CL()
                    roots = np.roots(Den)
                    if any(np.round(abs(roots),10)>round(0.99999999999999999999,10)):
                        sp_error = -10
                        tr_error = -10
                        L2_error = -10
                        S[0][k][Resolution - j - 1][i]=- 10
                    else:
                        system = control.TransferFunction(Num, Den, Ts)
                        system = control.minreal(system)
                        system = system.returnScipySignalLTI()[0][0]

                        T,yout = signal.dlsim(system, ref, t=t)
                        yout = yout.T[0]

                        error = ref - yout
                        tr_error = np.max((error[0:int((1/Ts * 0.807))]))
                        sp_error = np.max((error[int((1/Ts * 0.807))+1:-1]))
                        L2_error = 0
                        for er in error:
                            L2_error = L2_error + er**2 * Ts
                    
                    S[1][k][Resolution - j - 1][i]=L2_error
                    S[2][k][Resolution - j - 1][i]=tr_error
                    S[3][k][Resolution - j - 1][i]=sp_error
                    bar()

        np.save("Performance\Data_DT\Data_"+ Controller + "_DO_" +str(Do) + "_"+str(Resolution) + '.npy', S)