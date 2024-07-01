
import matplotlib.pyplot as plt

def stance_leg3D_scatter(ax, x, y, it, plus, minus, c1, c2):
    ax.scatter(x[plus], 
          y[plus], 
          it[plus], 
          c=c1, label='stance_leg = 1')
    ax.scatter(x[minus], 
          y[minus], 
          it[minus], 
          c=c2, label='stance_leg = -1')
