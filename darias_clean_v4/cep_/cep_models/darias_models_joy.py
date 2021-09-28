import torch
from cep_.utils import eul2rot

from cep_.kinematics import DarIASArm
from cep_.controllers import EBMControl, EnergyTree

from cep_ import maps
from cep_ import energies

# GPU_ON = torch.cuda.is_available()
GPU_ON = False
if GPU_ON:
    device = torch.device("cuda:0")
else:
    device = torch.device("cpu")


def cep_models_joy():
    #########################
    # get all the FK maps
    darias_kin = DarIASArm()

    #########################
    # End Effector Branch

    b = torch.Tensor([0.4, 0.1, 1.2])  # Desired Position
    R = eul2rot(torch.Tensor([-1.57, -1.57, 0.]))  # Desired orientation

    H = torch.eye(4)
    H[:3, :3] = R
    H[:3, -1] = b

    A = torch.eye(6)

    ee_joy_leaf = energies.TaskGoTO_JoyControl(dim=6, b=b, A=A, R=H, var=torch.eye(6) * 1.)
    pick_map = maps.Joy_SelectionMap(idx=6)
    ee_energy_tree = EnergyTree(branches=[ee_joy_leaf], map=pick_map, name="ee_energy_tree")

    fk_map = maps.Joy_FK_ALL(darias_kin)

    q_branches = [ee_energy_tree]
    energy_tree = EnergyTree(branches=q_branches, map=fk_map, name="energy_tree").to(device)
    policy = EBMControl(energy_tree=energy_tree, device=device, optimization_steps=5, dt=0.005, n_particles=10000)

    return policy