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
Controller = 'PID'
Do = False
Ts = 0.01


CL_Model = Model.DT_System(Controller,Do,m,wc,Ts)

Num, Den = CL_Model.PID()



print(Num)
print(Den)
