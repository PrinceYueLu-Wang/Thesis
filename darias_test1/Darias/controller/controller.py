import numpy as np

from cep_.utils import numpy2torch, torch2numpy
from cep_.liegroups.numpy import SO3, SE3

def ControllSpeed_eef(T_target,T_body):

    T_target_body=np.linalg.solve(T_target,T_body)

    Xe = SE3.from_matrix(T_target_body, normalize=True) 

    xtl = Xe.log()  
    vtl = -xtl

    A = SE3.from_matrix(T_target)
    Adj_lw = A.adjoint()
    ve_world = np.matmul(Adj_lw, vtl)
    
    return ve_world

def LineContSpeed_eef(T_target,T_body):
    
    vector1=T_target[0:3,-1]
    vector2=T_body[0:3,-1]

    v =vector1-vector2
    v_sclar=np.linalg.norm(v)
    v_unit=v/v_sclar

    v_unit=np.concatenate((v_unit,np.array([0.,0.,0.])))

    return v_unit





    
