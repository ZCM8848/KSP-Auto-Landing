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

def A(w):
    _A_ = np.empty([6,6])
    np.copyto(_A_[0:3,0:3] , np.zeros((3,3))     ) # top left
    np.copyto(_A_[0:3,3:6] , np.eye(3)           ) # top right
    np.copyto(_A_[3:6,0:3] , -np.square(S(w))    ) # bottom left
    np.copyto(_A_[3:6,3:6] , np.multiply(-1,S(w))) # bottom right
    return _A_