import numpy as np

def DistHomoMatrix(T1,T2):
    '''
    Args:
        T1 :[numpy.array]--4x4 homogeous matrix 
        T2 :[numpy.array]--4x4 homogeous matrix 

    Return:
        euclidian distance between T1 and T2
    '''

    v1=T1[0:3,-1]
    v2=T2[0:3,-1]

    return np.linalg.norm(v1-v2)