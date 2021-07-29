import numpy as np

from cep.utils import numpy2torch, torch2numpy
from cep.liegroups.numpy import SO3, SE3

def ControllSpeed_eef(T_target,T_body):

    T_target_body=np.linalg.solve(T_target,T_body)

    Xe = SE3.from_matrix(T_target_body, normalize=True) 

    xtl = Xe.log()  
    vtl = -xtl

    A = SE3.from_matrix(T_target)
    Adj_lw = A.adjoint()
    ve_world = np.matmul(Adj_lw, vtl)
    
    return ve_world





    
