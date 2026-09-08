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

def normalize_angle(angle, reference_angle):
    # Normalize angle to be closest to reference_angle, accounting for 2π periodicity
    return angle - 2 * np.pi * np.round((angle - reference_angle) / (2 * np.pi))

wrap_mask = [True, True, True, True, True, True]  # e.g., only wrap joints 1-5

def normalize_joint_angles(current_angles, new_angles, wrap_mask):
    normalized = []
    for i, (new, ref) in enumerate(zip(new_angles, current_angles)):
        if wrap_mask[i]:
            normalized.append(normalize_angle(new, ref))
        else:
            normalized.append(new)  # preserve absolute angle
    return np.array(normalized)

def choose_best_ik(prev_angles, pos, rot, a_n, d_n, b, tp):
    # Solution to replicate human arm motion (Selects elbow-down solution)
    combs1 = [1, 0, 1, 0] 
    combs2 = [0, 0, 1, 1]

    ik_solutions = []

    for i in range(4):
        solved_ik = solver(prev_angles, pos, rot, a_n, d_n, b, tp, combs2[i], combs1[i])
        # print(np.rad2deg(solved_ik))
        ik_solutions.append(solved_ik)

    best_solution = None
    min_rotation = float('inf')

    for solution in ik_solutions:
        # Normalize solution angles to be close to prev_angles
        normalized_solution = normalize_joint_angles(prev_angles, solution, wrap_mask)
        # Compute total rotation using normalized angles
        total_rotation = np.sum(np.abs(np.array(normalized_solution) - np.array(prev_angles)))
        if total_rotation < min_rotation:
            min_rotation = total_rotation
            best_solution = normalized_solution
    
    # If no valid solution is found, return previous angles
    if best_solution is None:
        return prev_angles
    
    return best_solution

def forward_kinematics(angles, a_n, d_n, b, tp):
        Th1 = angles[0]
        Th2 = angles[1]
        Th3 = angles[2]
        Th4 = angles[3]
        Th5 = angles[4]
        Th6 = angles[5]

        a_2 = a_n[1]
        a_3 = a_n[2]
        d_4 = d_n[3]
        d_5 = d_n[4]

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

        s_1 = sin(Th1)
        c_1 = cos(Th1)

        Th01 = [
            [c_1, -s_1, 0, 0],
            [s_1,  c_1, 0, 0],
            [ 0 ,   0 , 1, 0],
            [ 0 ,   0 , 0, 1]
        ]

        s_2 = sin(Th2)
        c_2 = cos(Th2)

        Th12 = [
            [-s_2, -c_2,  0, 0],
            [  0 ,   0 , -1, 0],
            [ c_2, -s_2,  0, 0],
            [  0 ,   0 ,  0, 1]
        ]

        s_3 = sin(Th3)
        c_3 = cos(Th3)

        Th23 = [
            [c_3, -s_3, 0, a_2],
            [s_3,  c_3, 0,  0 ],
            [ 0 ,   0 , 1,  0 ],
            [ 0 ,   0 , 0,  1 ]
        ]

        s_4 = sin(Th4)
        c_4 = cos(Th4)

        Th34 = [
            [ s_4, c_4, 0, a_3],
            [-c_4, s_4, 0,  0 ],
            [  0 ,  0 , 1, d_4],
            [  0 ,  0 , 0,  1 ]
        ]

        s_5 = sin(Th5)
        c_5 = cos(Th5)

        Th45 = [
            [ c_5, -s_5, 0,  0 ],
            [  0 ,   0 , 1, d_5],
            [-s_5, -c_5, 0,  0 ],
            [  0 ,   0 , 0,  1 ]
        ]

        s_6 = sin(Th6)
        c_6 = cos(Th6)

        Th56 = [
            [c_6, -s_6,  0, 0],
            [ 0 ,   0 , -1, 0],
            [s_6,  c_6,  0, 0],
            [ 0 ,   0 ,  0, 1]
        ]

        T0 = np.dot(T_0b, Th01)
        T1 = np.dot(T0, Th12)
        T2 = np.dot(T1, Th23)
        T3 = np.dot(T2, Th34)
        T4 = np.dot(T3, Th45)
        T5 = np.dot(T4, Th56)
        fpk = np.dot(T5, T_tp6)

        x = fpk[0][3]
        y = fpk[1][3]
        z = fpk[2][3]
        hand_pos = [x, y, z]
        return hand_pos