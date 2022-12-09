import numpy as np
import Model
import pathgenerator
import scipy.signal as signal
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from alive_progress import alive_bar; import time
import matplotlib
import control


m = 1
wc = 62.8
Controller = 'PDF'
Do = False

Ts = 0.001

CT = Model.CT_System(Controller,Do,m,wc,a1=1,y=0.8)
CT_CT = Model.CT_System(Controller,Do,m,wc,a1=0.3,y=0.8)

Num1,Den1 = CT.CL()

Num2,Den2 = CT_CT.CL()

pg1 = pathgenerator.PathGenerator()
pg1.max_s = 10
pg1.max_j = 1000
pg1.max_a = 100
pg1.max_v = 100

pg1.calculate_path(startpos=0.0, stroke=0.05, stroke_time = 0.707 ,starttime=0.1, fraction_s = 0.0, fraction_j= 1.0, fraction_a=1.0, fraction_v=1.0)
t = np.arange(0.0, 2.0, Ts)
ref=[]
for time_instant in t:
    ref.append(pg1.generate_x(time_instant))

fig , (ax1,ax2) = plt.subplots(1,2)

system = control.TransferFunction(Num2, Den2)
system = control.minreal(system)
system = system.returnScipySignalLTI()[0][0]
T,yout,xout = signal.lsim(system, ref,t)
yout = yout
error = ref - yout
ax1.plot(t,yout, 'r')
ax2.plot(t,error, 'r')

system = control.TransferFunction(Num1, Den1)
system = control.minreal(system)
system = system.returnScipySignalLTI()[0][0]
T,yout,xout = signal.lsim(system, ref,t)
yout = yout
error = ref - yout

ax1.plot(t,yout, '#1f77b4')
ax2.plot(t,error, '#1f77b4')

ax1.set_title('Postion')
ax2.set_title('Error')

ax1.set_ylabel(r'$y$ [m]')
ax1.set_xlabel(r'$t$ [s]')
ax2.set_ylabel(r'$e$ [m]')
ax2.set_xlabel(r'$t$ [s]')
ax2.legend([r'$\alpha=0.3, \gamma =0.8$',r'$\alpha=1.0, \gamma =0.8$'])
plt.tight_layout()

plt.savefig('Figures/'+str(Controller), dpi=1200)
plt.show()
