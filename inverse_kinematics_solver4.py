import time
import math
from math import cos as cos
from math import sin as sin
from math import atan2 as atan2
from math import acos as acos
from math import asin as asin
from math import sqrt as sqrt
from math import atan as atan
from math import pi as pi
import numpy as np

def generate_rot_matrix(alpha, beta, gamma):
    rot_matrix = np.array([
        [cos(alpha)*cos(beta), cos(alpha)*sin(beta)*sin(gamma) - cos(gamma)*sin(alpha), cos(alpha)*cos(gamma)*sin(beta) + sin(alpha)*sin(gamma)],
        [cos(beta)*sin(alpha), cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma), cos(gamma)*sin(alpha)*sin(beta) - cos(alpha)*sin(gamma)],
        [     -sin(beta),                        cos(beta)*sin(gamma),                                     cos(beta)*cos(gamma)                ]
    ])
    return rot_matrix

def solver(prev_angles, pos, rot, a_n, d_n, b, tp, t1_v, t2_v):
    if t1_v:
        t1_m = -1
    else:
        t1_m = 1

    if t2_v:
        t2_m = -1
    else:
        t2_m = 1
    ### INPUT PARAMETERS ###
    x = pos[0]
    y = pos[1]
    z = pos[2]

    # alpha = rot[0]
    # beta = rot[1]
    # gamma = rot[2]
    ### INPUT PARAMETERS ###

    ### CONSTANTS ###

    T_0b = [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, b],
        [0, 0, 0, 1]
    ]

    T_tp6 = [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, tp],
        [0, 0, 0, 1]
    ]
    ### CONSTANTS ###

    Tb_tp = [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ]

    Tb_tp = np.array(Tb_tp, dtype=float)

    Tb_tp[0][3] = x
    Tb_tp[1][3] = y
    Tb_tp[2][3] = z
    Tb_tp[:3, :3] = rot

    Tb_tp = np.round(Tb_tp, 4)

    T_06 = np.dot(np.linalg.inv(T_0b),Tb_tp)
    T_06 = np.dot(T_06,np.linalg.inv(T_tp6))

    E1 = T_06[1][3]
    F1 = -T_06[0][3]
    G1 = d_n[3]

    t1_sqrt = pow(E1,2) + pow(F1,2) - pow(G1,2)
    try:
        t1 = (-F1 + t1_m * sqrt(t1_sqrt))/(G1 - E1)
        Th1 = 2*atan(t1)
    except ValueError as e:
        return prev_angles

    th6_atan1 = T_06[0][1]*sin(Th1) - T_06[1][1]*cos(Th1)
    th6_atan2 = T_06[1][0]*cos(Th1) - T_06[0][0]*sin(Th1)
    try:
        Th6 = atan2(th6_atan1, th6_atan2)
    except ValueError as e:
        return prev_angles
    
    th5_atan1 = (T_06[1][0]*cos(Th1) - T_06[0][0]*sin(Th1))*cos(Th6) + (T_06[0][1]*sin(Th1) - T_06[1][1]*cos(Th1))*sin(Th6)
    th5_atan2 = T_06[0][2]*sin(Th1) - T_06[1][2]*cos(Th1)
    try:
        Th5 = atan2(th5_atan1, th5_atan2)
    except ValueError as e:
        return prev_angles

    A1 = (T_06[2][0]*cos(Th6) - T_06[2][1]*sin(Th6))/cos(Th5)
    B1 = T_06[2][1]*cos(Th6) + T_06[2][0]*sin(Th6)
    a1 = -T_06[0][3]*cos(Th1) - T_06[1][3]*sin(Th1) - d_n[4]*A1
    b1 = T_06[2][3] - d_n[4]*B1
    E2 = -2 * a_n[1] * b1
    F2 = -2 * a_n[1] * a1
    G2 = pow(a_n[1],2) + pow(a1,2) + pow(b1,2) - pow(a_n[2],2)
    t2_sqrt = pow(E2,2) + pow(F2,2) - pow(G2,2)

    try:
        t2 = (-F2 + t2_m * sqrt(t2_sqrt))/(G2 - E2)
        Th2 = 2*atan(t2)
    except ValueError as e:
        return prev_angles
    
    try:
        Th3 = atan2(a1 - a_n[1]*sin(Th2), b1 - a_n[1]*cos(Th2)) - Th2
    except ValueError as e:
        return prev_angles

    try:
        Th4 = atan2(A1, B1) - Th2 - Th3
    except ValueError as e:
        return prev_angles

    return [Th1,Th2,Th3,Th4,Th5,Th6]


def choose_best_ik(prev_angles, pos, rot, a_n, d_n, b, tp):
    combs1 = [1,0,1,0]
    combs2 = [0,0,1,1]

    ik_solutions = []

    for i in range(4):
        solved_ik = solver(prev_angles, pos, rot, a_n, d_n, b, tp, combs2[i], combs1[i])
        # solved_ik = [x * 180/pi for x in solved_ik]

        ik_solutions.append(solved_ik)

    best_solution = None
    min_rotation = float('inf')

    for solution in ik_solutions:
        total_rotation = np.sum(np.abs(np.array(solution) - np.array(prev_angles)))
        if total_rotation < min_rotation:
            min_rotation = total_rotation
            best_solution = solution
    
    return best_solution

def normalize_angle(angle, reference_angle):
    # Normalize angle to be closest to reference_angle, accounting for 2π periodicity
    return angle - 2 * np.pi * np.round((angle - reference_angle) / (2 * np.pi))

def normalize_joint_angles(current_angles, new_angles):
    # Normalize all joint angles to be close to current angles
    return np.array([normalize_angle(new, ref) for new, ref in zip(new_angles, current_angles)])

def choose_best_ik(prev_angles, pos, rot, a_n, d_n, b, tp):
    combs1 = [1, 0, 1, 0]
    combs2 = [0, 0, 1, 1]

    ik_solutions = []

    for i in range(4):
        solved_ik = solver(prev_angles, pos, rot, a_n, d_n, b, tp, combs2[i], combs1[i])
        ik_solutions.append(solved_ik)

    best_solution = None
    min_rotation = float('inf')

    for solution in ik_solutions:
        # Normalize solution angles to be close to prev_angles
        normalized_solution = normalize_joint_angles(prev_angles, solution)
        # Compute total rotation using normalized angles
        total_rotation = np.sum(np.abs(np.array(normalized_solution) - np.array(prev_angles)))
        if total_rotation < min_rotation:
            min_rotation = total_rotation
            best_solution = normalized_solution
    
    # If no valid solution is found, return previous angles
    if best_solution is None:
        return prev_angles
    
    return best_solution