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

# Robot geometry
ROBOT_A = [0.0, 0.4251, 0.39215, 0.0, 0.0, 0.0]
ROBOT_D = [0.0, 0.0, 0.0, 0.11000, 0.09475, 0.0]
ROBOT_B = 0.0892
ROBOT_TP = 0.07495

# Safety clearance
MIN_CLEARANCE = 0.045
PROTECTIVE_STOP_DISTANCE = 0.028 # from ursim

JOINT_MIN = np.deg2rad([-360.0, -360.0, -360.0, -360.0, -360.0, -360.0])
JOINT_MAX = np.deg2rad([360.0, 360.0, 360.0, 360.0, 360.0, 360.0])

wrap_mask = [True, True, True, True, True, True]  # e.g., only wrap joints 1-5

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

    # Validate inputs
    try:
        prev_angles = np.asarray(prev_angles, dtype=float)
        pos = np.asarray(pos, dtype=float)
        rot = np.asarray(rot, dtype=float)
        a_n = np.asarray(a_n, dtype=float)
        d_n = np.asarray(d_n, dtype=float)
    except Exception:
        return None

    if (prev_angles.shape != (6,) or pos.shape != (3,) or rot.shape != (3, 3) or len(a_n) < 3 or len(d_n) < 5):
        return None

    if not np.all(np.isfinite(pos)):
        return None
    if not np.all(np.isfinite(rot)):
        return None
    
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

    if t1_sqrt < 0.0:
        if t1_sqrt > -1e-10:
            t1_sqrt = 0.0
        else:
            return None

    denominator = G1 - E1
    if abs(denominator) < 1e-12:
        return None

    try:
        t1 = (-F1 + t1_m * sqrt(t1_sqrt))/(G1 - E1)
        Th1 = 2*atan(t1)
    except (ValueError, ZeroDivisionError, FloatingPointError):
        return None

    th6_atan1 = T_06[0][1]*sin(Th1) - T_06[1][1]*cos(Th1)
    th6_atan2 = T_06[1][0]*cos(Th1) - T_06[0][0]*sin(Th1)
    try:
        Th6 = atan2(th6_atan1, th6_atan2)
    except (ValueError, FloatingPointError):
        return None
    
    th5_atan1 = (T_06[1][0]*cos(Th1) - T_06[0][0]*sin(Th1))*cos(Th6) + (T_06[0][1]*sin(Th1) - T_06[1][1]*cos(Th1))*sin(Th6)
    th5_atan2 = T_06[0][2]*sin(Th1) - T_06[1][2]*cos(Th1)
    try:
        Th5 = atan2(th5_atan1, th5_atan2)
    except (ValueError, FloatingPointError):
        return None

    cos_Th5 = cos(Th5)
    if abs(cos_Th5) < 1e-10:
        return None
    
    A1 = (T_06[2][0]*cos(Th6) - T_06[2][1]*sin(Th6))/cos(Th5)
    B1 = T_06[2][1]*cos(Th6) + T_06[2][0]*sin(Th6)
    a1 = -T_06[0][3]*cos(Th1) - T_06[1][3]*sin(Th1) - d_n[4]*A1
    b1 = T_06[2][3] - d_n[4]*B1
    E2 = -2 * a_n[1] * b1
    F2 = -2 * a_n[1] * a1
    G2 = pow(a_n[1],2) + pow(a1,2) + pow(b1,2) - pow(a_n[2],2)
    t2_sqrt = pow(E2,2) + pow(F2,2) - pow(G2,2)

    if t2_sqrt < 0.0:
        if t2_sqrt > -1e-10:
            t2_sqrt = 0.0
        else:
            return None

    denominator = G2 - E2
    if abs(denominator) < 1e-12:
        return None
    
    try:
        t2 = (-F2 + t2_m * sqrt(t2_sqrt))/(G2 - E2)
        Th2 = 2*atan(t2)
    except (ValueError, ZeroDivisionError, FloatingPointError):
        return None
    
    try:
        Th3 = atan2(a1 - a_n[1]*sin(Th2), b1 - a_n[1]*cos(Th2)) - Th2
    except (ValueError, FloatingPointError):
        return None

    try:
        Th4 = atan2(A1, B1) - Th2 - Th3
    except (ValueError, FloatingPointError):
        return None

    solution = np.array([Th1,Th2,Th3,Th4,Th5,Th6], dtype=float)
    if not np.all(np.isfinite(solution)):
        return None
    
    return solution

def normalize_angle(angle, reference_angle):
    # Normalize angle to be closest to reference_angle, accounting for 2π periodicity
    return angle - 2 * np.pi * np.round((angle - reference_angle) / (2 * np.pi))

def normalize_joint_angles(current_angles, new_angles, wrap_mask):
    current_angles = np.asarray(current_angles, dtype=float)
    new_angles = np.asarray(new_angles, dtype=float)
    normalized = np.empty(6, dtype=float)
    for i, (new, ref) in enumerate(zip(new_angles, current_angles)):
        if wrap_mask[i]:
            normalized[i] = (normalize_angle(new, ref))
        else:
            normalized[i] = new  # preserve absolute angle
    return normalized

def within_joint_limits(angles, joint_min=JOINT_MIN, joint_max=JOINT_MAX):
    angles = np.asarray(angles, dtype=float)
    if angles.shape != (6,):
        return False
    if not np.all(np.isfinite(angles)):
        return False
    return bool(np.all(angles >= joint_min) and np.all(angles <= joint_max))

def forward_kinematics_links(angles, a_n=ROBOT_A, d_n=ROBOT_D, b=ROBOT_B, tp=ROBOT_TP):
        angles = np.asarray(angles, dtype=float)
        if angles.shape != (6,):
            raise ValueError("angles must contain exactly six joint angles.")
        if not np.all(np.isfinite(angles)):
            raise ValueError("angles contain non-finite values.")
        
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
        T6 = np.dot(T5, T_tp6)
        T_tcp = np.dot(T6, T_tp6)

        return {
            "T0": T0,
            "T1": T1,
            "T2": T2,
            "T3": T3,
            "T4": T4,
            "T5": T5,
            "T6": T6,
            "T_tcp": T_tcp }

        # x = fpk[0][3]
        # y = fpk[1][3]
        # z = fpk[2][3]
        # hand_pos = [x, y, z]
        # return hand_pos
def forward_kinematics(angles, a_n=ROBOT_A, d_n=ROBOT_D, b=ROBOT_B, tp=ROBOT_TP):
    frames = forward_kinematics_links(angles, a_n, d_n, b, tp)
    tcp = frames["T_tcp"]
    return [tcp[0, 3], tcp[1, 3], tcp[2, 3]]

def point_to_segment_distance(point, segment_start, segment_end):
    point = np.asarray(point, dtype=float)
    segment_start = np.asarray(segment_start, dtype=float)
    segment_end = np.asarray(segment_end, dtype=float)

    segment = segment_end - segment_start

    segment_length_squared = np.dot(segment, segment)

    if segment_length_squared < 1e-12:
        return float(np.linalg.norm(point - segment_start))

    projection = np.dot(point - segment_start, segment) / segment_length_squared

    projection = np.clip(projection, 0.0, 1.0)

    closest_point = (segment_start + projection * segment)
    distance = np.linalg.norm(point - closest_point)

    return float(distance)

def calculate_flange_lower_arm_clearance(angles, a_n=ROBOT_A, d_n=ROBOT_D, b=ROBOT_B, tp=ROBOT_TP):
    frames = forward_kinematics_links(angles, a_n, d_n, b, tp)

    T2 = frames["T2"]
    T3 = frames["T3"]
    T5 = frames["T5"]

    lower_arm_start = T2[:3, 3]
    lower_arm_end = T3[:3, 3]

    flange_position = T5[:3, 3]

    clearance = point_to_segment_distance(flange_position, lower_arm_start, lower_arm_end)

    return clearance

def is_clearance_safe(angles, min_clearance=MIN_CLEARANCE, a_n=ROBOT_A, d_n=ROBOT_D, b=ROBOT_B, tp=ROBOT_TP):
    try:
        clearance = calculate_flange_lower_arm_clearance(angles, a_n, d_n, b, tp)
    except Exception:
        return False, 0.0

    safe = (np.isfinite(clearance) and clearance >= min_clearance)

    return bool(safe), float(clearance)

def choose_best_ik(prev_angles, pos, rot, a_n=ROBOT_A, d_n=ROBOT_D, b=ROBOT_B, tp=ROBOT_TP):
    prev_angles = np.asarray(prev_angles, dtype=float)
    if prev_angles.shape != (6,):
        raise ValueError("prev_angles must contain six joint angles.")
    if not np.all(np.isfinite(prev_angles)):
        raise ValueError("prev_angles contains non-finite values.")
    
    # Solution to replicate human arm motion (Selects elbow-down solution)
    combs1 = [1, 0, 1, 0] 
    combs2 = [0, 0, 1, 1]

    ik_solutions = []

    print("\nIK SAFETY CHECK")
    print(f"Required flange/lower-arm clearance: " f"{min_clearance * 1000:.1f} mm")

    for i in range(4):
        solved_ik = solver(prev_angles, pos, rot, a_n, d_n, b, tp, combs2[i], combs1[i])
        if solved_ik is None:
            print(f"Candidate {i + 1}: " f"IK FAILED")
            continue
        normalized_solution = normalize_joint_angles(prev_angles, solved_ik, wrap_mask)
        if not np.all(np.isfinite(normalized_solution)):
            print(f"Candidate {i + 1}: " f"REJECTED - non-finite angles")
            continue
        if not within_joint_limits(normalized_solution):
            print(f"Candidate {i + 1}: " f"REJECTED - joint limit")
            continue
        safe, clearance = is_clearance_safe(normalized_solution, min_clearance, a_n, d_n, b, tp)
        total_rotation = np.sum(np.abs(normalized_solution - prev_angles))
        if not safe:
            print(f"Candidate {i + 1}: " f"REJECTED - clearance " f"{clearance * 1000.:1f} mm")
            ik_solutions.append({"angles": normalized_solution,
                                 "clearance": clearance,
                                 "total_rotation": total_rotation,
                                 "safe": False})
            continue

        print(f"Candidate {i + 1}: " 
              f"SAFE - clearance " 
              f"{clearance * 1000:.1f} mm, " 
              f"joint movement "
              f"{np.rad2deg(total_rotation):.2f} deg")
        ik_solutions.append({"angles": normalized_solution,
                             "clearance": clearance,
                             "total_rotation": total_rotation,
                             "safe": True}})

        safe_candidates = [candidate for candidate in ik_solutions if candidate["safe"]]
        if not safe_candidates:
            print("\nIK RESULT: "
                  "NO SAFE IK SOLUTION")
            try:
                previous_clearance = (calculate_flange_lower_arm_clearance(prev_angles, a_n, d_n, b, tp))
                print("Fallback configuration clearance: "
                      f"{previous_clearance * 1000:.1f} mm")
            except Exception:
                print("Fallback configuration clearance: "
                      "unable to calculate")
        print("Returning previous robot configuration.")
        return prev_angles.copy()
        # print(np.rad2deg(solved_ik))
        # ik_solutions.append(solved_ik)

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