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
wc = 63.8
Controller = 'PD'
Do = True
Ts = 0.1
with alive_bar(3*Resolution**2) as bar:
    S0=np.zeros((Resolution,3*Resolution), dtype = int)
    S = S0.copy()    
    CL_Model = Model.DT_System(Controller,Do,m,wc,Ts,Bi_DO=0.0001)
    for l in range(0,Resolution):
        y = l/Resolution + 0.01
        for n in range(0,3*Resolution):
            a1 = n/Resolution - 1
            

            CL_Model.Update(wc, Ts , a1 = a1, y = y,Bi_DO=0.0001)



            Num, Den = CL_Model.CL()
            roots = np.roots(Den)

            if any(abs(roots)>1.000004):

                S[Resolution - l - 1][n]= -2

            bar()
D2_map = S





fig, ax = plt.subplots()
CMAP=plt.cm.RdYlGn
ax.matshow(D2_map, cmap=CMAP, vmin= -2 , vmax=2, aspect='auto', extent=(-1,2-(1/Resolution),0,1.01-(1/Resolution)))
plt.gcf().set_facecolor("white")
# Axes
ax.xaxis.tick_bottom()
ax.set(xlabel='α', ylabel='γ')
plt.xticks(np.arange(-1, 2.5, 0.5)) 
plt.yticks(np.arange(0, 1.1, 0.1)) 
# Grid and ticker second x-grid line
ax.grid(which='both', ls='--') # , color='#4CA3DD'
lines = ax.get_xgridlines()
line2=lines[2]
line2.set_linewidth(2)
# Legend and title, depending on comparing or not, approximately right colors
custom_lines = [Patch(facecolor=CMAP(0.5), edgecolor='k', label='Stable'),
                Patch(facecolor=CMAP(0.125), edgecolor='k', label='Unstable')]
# Overlay a single scatter plot to show the current type stability
ax.legend(handles=custom_lines,loc='upper right')
plt.show()



