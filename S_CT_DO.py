import numpy as np
import Model
import pathgenerator
import scipy.signal as signal
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from alive_progress import alive_bar; import time
import matplotlib

Resolution = 15
m = 1
wc = 10


with alive_bar(4*3*Resolution**3) as bar:
    Do = True
    for Controller in ['PD', 'PDF', 'PID', 'PIDF']:
        S0=np.zeros((Resolution,Resolution,Resolution*3,Resolution,3*Resolution), dtype = int)
        S = S0.copy()    
        CL_Model = Model.CT_System(Controller,Do,m,wc)
        for o in range(0,Resolution):
            Bi_Do = np.logspace(-3,0.3,num=Resolution)[o]
            for i in range(0,Resolution):
                r = i/Resolution + 0.01
                for j in range(0,3*Resolution):
                    a2 = j/Resolution - 1
                    for l in range(0,Resolution):
                        y = l/Resolution + 0.01
                        for n in range(0,3*Resolution):
                            a1 = n/Resolution - 1

                            CL_Model.Update(a1 = a1, y = y, a2 = a2, r = r, Bi_DO = Bi_Do)

                            Num, Den = CL_Model.CL()
                            roots = np.roots(Den)

                            if any(roots.real>0):
                                S[o][i][j][Resolution - l - 1][n]= -2

                    bar()

        np.save("Stability\Data_CT_DO\Data_"+ Controller + "_DO_" +str(Do) + "_"+str(Resolution) + '.npy', S)

