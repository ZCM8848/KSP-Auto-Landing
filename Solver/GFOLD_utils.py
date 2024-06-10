import numpy as np

def e(i): # create a specific basis vector
    if i==0:
        return [1,0,0]
    if i==1:
        return [0,1,0]
    if i==2:
        return [0,0,1]

def S_mat(_w_): # _w_ to distinguish from our global namespace's w!
    return np.matrix([[0,-_w_[2],+_w_[1]],
                     [_w_[2],0, -_w_[0]],
                     [-_w_[1],_w_[0],0]])