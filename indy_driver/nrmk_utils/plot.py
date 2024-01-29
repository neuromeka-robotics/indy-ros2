import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as mplot3d

def scatter_3d(X, style='.', view=None, xlabel="x",ylabel="y", zlabel="z",
               sub=None, fig=None):

    if sub is None:
        fig = fig or plt.figure(figsize=(15, 15))
    sub = sub or fig.add_subplot(1, 1, 1, projection="3d")
    X = list(X)
    X0 = np.array(X[0:1])
    assert 2<=len(X0.shape)<=3, "data dimension should be 2 or 3"
    if len(X0.shape)==2:
        X = np.array([X])
    for X_i in X:
        x, y, z = np.transpose(X_i)
        sub.plot(x, y, z, style)

    if view is not None:
        sub.view_init(*view)
    sub.set_xlabel(xlabel)
    sub.set_ylabel(ylabel)
    sub.set_zlabel(zlabel)
    # sub.axis('equal')