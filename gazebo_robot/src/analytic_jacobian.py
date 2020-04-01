# -*- coding: utf-8 -*-
"""
Created on Thu Feb 20 14:34:49 2020

@author: yuzhe
"""

import sympy as sp

q_1, q_2, q_3, q_4, q_5, q_6 = sp.symbols('q_1 q_2 q_3 q_4 q_5 q_6')
d_1, d_2, d_3, d_4, d_5, d_6 = sp.symbols('d_1, d_2, d_3, d_4, d_5, d_6')
a_1, a_2, a_3, a_4, a_5, a_6 = sp.symbols('a_1, a_2, a_3, a_4, a_5, a_6')
al_1, al_2, al_3, al_4, al_5, al_6 = sp.symbols('al_1, al_2,al_3, al_4, al_5, al_6')

joint_num = 6


def Transfo_Mat():
    A_pre = sp.eye(4)  # initialize w^T_w as intendify
    A = []

    for i in range(joint_num):
        """j = str(i)  previous joint number"""
        k = str(i + 1)

        R_theta = sp.Matrix([[sp.cos("q_" + k), -sp.sin("q_" + k), 0, 0],
                             [sp.sin("q_" + k), sp.cos("q_" + k), 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])

        T_d = sp.Matrix([[1, 0, 0, 0],
                         [0, 1, 0, 0],
                         [0, 0, 1, "d_" + k],
                         [0, 0, 0, 1]])

        T_a = sp.Matrix([[1, 0, 0, "a_" + k],
                         [0, 1, 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])

        R_alpha = sp.Matrix([[1, 0, 0, 0],
                             [0, sp.cos("al_" + k), -sp.sin("al_" + k), 0],
                             [0, sp.sin("al_" + k), sp.cos("al_" + k), 0],
                             [0, 0, 0, 1]])

        A_jk = R_theta * T_d * T_a * R_alpha
        A_pre = A_pre * A_jk  # w^Transform_i = w^Transform_i-1 * i-1^Transform_i
        # print("A_{}{} =\n{}\n\n{}\n\n{}\n\n{}\n\n".format(0, k, A_pre[0, :], A_pre[1, :], A_pre[2, :], A_pre[3, :]))
        A.append(A_pre)

    return A


def Jacobian_Mat(transfor_mat):
    """differentiate translation vector of Transfo_Mat to get J_v(q_i)"""
    jacob_v = []
    t = transfor_mat[joint_num - 1][0:3, 3]  # translation vector of the last joint's Transfo_Mat

    for j in range(3):
        for k in range(joint_num):
            jacob_v_jk = sp.diff(t[j], "q_" + str(k + 1))
            jacob_v.append(jacob_v_jk)
    jacobian_v = sp.Array(jacob_v).reshape(3, joint_num)

    """calculate the rotation axis from rotation matrix and diff. it to get J_omega(q_i)"""
    jacob_omega = []
    R = transfor_mat[joint_num - 1][0:3, 0:3]  # rotation matrix of the last joint's Transfo_Mat

    # https://en.wikipedia.org/wiki/Rotation_matrix#Conversion_from_and_to_axis%E2%80%93angle
    # N(axes) = 2 * sin(theta)    theta / N(axes) = 0.5 * theta/sin(theta)  ?????
    axes = [0.5 * (R[2, 1] - R[1, 2]),
            0.5 * (R[0, 2] - R[2, 0]),
            0.5 * (R[1, 0] - R[0, 1])]

    # to get rotation angle theta around axes  1+2*cos(theta) = Tr(R)

    cos_theta = (R[0, 0] + R[1, 1] + R[2, 2] - 1) / 2
    # theta = sp.acos(cos_theta)
    coef = cos_theta / sp.sqrt(axes[0] ** 2 + axes[1] ** 2 + axes[2] ** 2)
    axes = [coef * axes[0], coef * axes[1], coef * axes[2]]

    # print("axes\n{}\n\n".format(axes))

    for j in range(3):
        for k in range(joint_num):
            jacob_omega_jk = sp.diff(axes[j], "q_" + str(k + 1))
            jacob_omega.append(jacob_omega_jk)
    jacobian_omega = sp.Array(jacob_omega).reshape(3, joint_num)

    jacobian = [jacobian_v, jacobian_omega]
    jacobian = sp.Array(jacobian).reshape(6, joint_num)

    # print("normal\n{}\nsimplify\n{}\ntrigsimp\n{}\n".format(jacobian[0, 0], sp.simplify(jacobian[0, 0]),
    #                                                         sp.trigsimp(jacobian[0, 0])))
    # print("Jacobian Matrix is\n{}\n\n{}\n\n{}\n\n{}\n\n{}\n\n{}".format(jacobian[0, :], jacobian[1, :], jacobian[2, :],
    #                                                          jacobian[3, :], jacobian[4, :], jacobian[5, :]))
    return jacobian


def substitution(transfor_mat, jacobian):
    theta_ = {"1": 0, "2": 0 + sp.pi / 2, "3": 0 + sp.pi / 2, "4": 0, "5": 0, "6": 0}
    d_ = {"1": 0, "2": 0, "3": 0, "4": 0.305, "5": 0, "6": 0}
    a_ = {"1": 0, "2": 0.350, "3": 0, "4": 0, "5": 0, "6": 0}
    alpha_ = {"1": sp.pi / 2, "2": sp.pi, "3": sp.pi / 2, "4": -sp.pi / 2, "5": sp.pi / 2, "6": 0}

    q_1, q_2, q_3, q_4, q_5, q_6 = sp.symbols('q_1 q_2 q_3 q_4 q_5 q_6')
    d_1, d_2, d_3, d_4, d_5, d_6 = sp.symbols('d_1, d_2, d_3, d_4, d_5, d_6')
    a_1, a_2, a_3, a_4, a_5, a_6 = sp.symbols('a_1, a_2, a_3, a_4, a_5, a_6')
    al_1, al_2, al_3, al_4, al_5, al_6 = sp.symbols('al_1, al_2,al_3, al_4, al_5, al_6')

    transfor_mat_06 = sp.Array(transfor_mat[joint_num - 1][:, :])

    transfo_mat_num = transfor_mat_06.subs([(q_1, theta_['1']), (d_1, d_['1']), (a_1, a_['1']), (al_1, alpha_['1']),
                                            (q_2, theta_['2']), (d_2, d_['2']), (a_2, a_['2']), (al_2, alpha_['2']),
                                            (q_3, theta_['3']), (d_3, d_['3']), (a_3, a_['3']), (al_3, alpha_['3']),
                                            (q_4, theta_['4']), (d_4, d_['4']), (a_4, a_['4']), (al_4, alpha_['4']),
                                            (q_5, theta_['5']), (d_5, d_['5']), (a_5, a_['5']), (al_5, alpha_['5']),
                                            (q_6, theta_['6']), (d_6, d_['6']), (a_6, a_['6']), (al_6, alpha_['6'])])

    jacobian_num = jacobian.subs([(q_1, theta_['1']), (d_1, d_['1']), (a_1, a_['1']), (al_1, alpha_['1']),
                                  (q_2, theta_['2']), (d_2, d_['2']), (a_2, a_['2']), (al_2, alpha_['2']),
                                  (q_3, theta_['3']), (d_3, d_['3']), (a_3, a_['3']), (al_3, alpha_['3']),
                                  (q_4, theta_['4']), (d_4, d_['4']), (a_4, a_['4']), (al_4, alpha_['4']),
                                  (q_5, theta_['5']), (d_5, d_['5']), (a_5, a_['5']), (al_5, alpha_['5']),
                                  (q_6, theta_['6']), (d_6, d_['6']), (a_6, a_['6']), (al_6, alpha_['6'])])

    print("Numerical Transformation Matrix is\n{}\nNumerical Jacobian Matrtix is\n{}".format(transfo_mat_num,
                                                                                             jacobian_num))

    return transfo_mat_num, jacobian_num


operators = ["+", "-", "*", "/", "(", ")", ";"]


def format_string_pow(s):
    pow_s = "**"
    index = s.find(pow_s)
    if index != -1:
        insert_id_s = 0
        if s[index - 1] == ')':
            closed = 0
            for i in range(index - 1, -1, -1):
                if s[i] == ')':
                    closed += 1
                elif s[i] == '(':
                    closed -= 1
                if closed == 0:
                    insert_id_s = i
                    break
        else:
            for i in range(index - 1, -1, -1):
                if s[i] in operators:
                    insert_id_s = i + 1
                    break

        insert_id_p = 0
        if s[index + 2] == '(':
            opened = 0
            for i in range(index + 2, len(s)):
                if s[i] == '(':
                    opened += 1
                elif s[i] == ')':
                    opened -= 1
                if opened == 0:
                    insert_id_p = i + 1
                    break
        else:
            for i in range(index + 2, len(s)):
                if s[i] in operators:
                    insert_id_p = i
                    break

        new_s = s[:insert_id_s] + "pow(" + s[insert_id_s:index] + "," + s[index + 2:insert_id_p] + ")" + s[insert_id_p:]

        return index, new_s
    else:
        return -1, s


def format_string_trig(s):
    new_s = s
    for i in range(6):
        new_s = new_s.replace(f"cos(q_{i+1})", f"c_[{i}]")
        new_s = new_s.replace(f"sin(q_{i+1})", f"s_[{i}]")
    return new_s


def transformation(t_z, t_y, o_r_y, r_y, r_z):
    A_pre = sp.eye(4)  # initialize w^T_w as intendify
    # A = []

    for i in range(len(t_z)):
        """j = str(i)  previous joint number"""
        k = str(i + 1)

        T_z = sp.Matrix([[1, 0, 0, 0],
                         [0, 1, 0, 0],
                         [0, 0, 1, t_z[i]],
                         [0, 0, 0, 1]])

        T_y = sp.Matrix([[1, 0, 0, 0],
                         [0, 1, 0, t_y[i]],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])

        o_R_y = sp.Matrix([[sp.cos(o_r_y[i]), 0, sp.sin(o_r_y[i]), 0],
                           [0, 1, 0, 0],
                           [-sp.sin(o_r_y[i]), 0, sp.cos(o_r_y[i]), 0],
                           [0, 0, 0, 1]])

        R_y = sp.Matrix([[sp.cos(r_y[i]), 0, sp.sin(r_y[i]), 0],
                         [0, 1, 0, 0],
                         [-sp.sin(r_y[i]), 0, sp.cos(r_y[i]), 0],
                         [0, 0, 0, 1]])

        R_z = sp.Matrix([[sp.cos(r_z[i]), -sp.sin(r_z[i]), 0, 0],
                         [sp.sin(r_z[i]), sp.cos(r_z[i]), 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])

        A_jk = T_z * T_y * o_R_y * R_y * R_z
        A_pre = A_pre * A_jk  # w^Transform_i = w^Transform_i-1 * i-1^Transform_i
        # print("A_{}{} =\n{}\n\n{}\n\n{}\n\n{}\n\n".format(0, k, A_pre[0, :], A_pre[1, :], A_pre[2, :], A_pre[3, :]))
        # A.append(A_pre)

    return A_pre


def jacobian(t):
    translation = t[0:3, 3]

    R = t[0:3, 0:3]
    S = 0.5 * (R - R.T)
    cos_theta = (R[0, 0] + R[1, 1] + R[2, 2] - 1) / 2
    sine_theta = sp.sqrt(1 - cos_theta ** 2)
    coef = sine_theta / sp.sqrt(S[2, 1] ** 2 + S[2, 0] ** 2 + S[0, 1] ** 2)
    rotation = [S[2, 1] * coef, S[2, 0] * coef, S[0, 1] * coef]

    jacobian_t = sp.zeros(3, 6)
    for i in range(3):
        for k in range(joint_num):
            jacobian_t[i, k] = sp.diff(translation[i], "q_" + str(k + 1))

    jacobian_r = sp.zeros(3, 6)
    for i in range(3):
        for k in range(joint_num):
            jacobian_r[i, k] = sp.diff(rotation[i], "q_" + str(k + 1))

    return jacobian_t, jacobian_r


def test():
    q_1, q_2, q_3, q_4, q_5, q_6 = sp.symbols('q_1 q_2 q_3 q_4 q_5 q_6')

    t_z = [0.05, 0.15, 0.15, 0.15, 0.15, 0.15, 0.1]
    t_y = [0, -0.1, 0, 0.1, 0, -0.1, 0]
    o_r_y = [0, -0.8, 0, 1.56, 0, 1.56, 0]
    r_y = [0, q_2, 0, q_4, 0, q_6, 0]
    r_z = [q_1, 0, q_3, 0, q_5, 0, 0]

    transform = transformation(t_z, t_y, o_r_y, r_y, r_z)

    j_t, j_r = jacobian(transform)

    theta_ = [0, 0, 0, 0, 0, 0]

    j_t_num = j_t.subs([(q_1, theta_[0]), (q_2, theta_[1]), (q_3, theta_[2]), (q_4, theta_[3]),
                        (q_5, theta_[4]), (q_6, theta_[5])])
    j_r_num = j_r.subs([(q_1, theta_[0]), (q_2, theta_[1]), (q_3, theta_[2]), (q_4, theta_[3]),
                        (q_5, theta_[4]), (q_6, theta_[5])])

    # t = transform.subs([(q_1, theta_[0]), (q_2, theta_[1]), (q_3, theta_[2]), (q_4, theta_[3]),
    #                     (q_5, theta_[4]), (q_6, theta_[5])])

    # for i in range(4):
    #     for j in range(4):
    #         s = "transform_({}, {}) = {};".format(i, j, transform[i, j])
    #         alt_s = format_string_trig(s)
    #         cont, alt_s = format_string_pow(alt_s)
    #         while cont > 0:
    #             cont, alt_s = format_string_pow(alt_s)
    #         print(alt_s)
    # for i in range(4):
    #     for j in range(4):
    #         s = "transform_({}, {}) = {};".format(i, j, sp.simplify(transform[i, j]))
    #         alt_s = format_string_trig(s)
    #         cont, alt_s = format_string_pow(alt_s)
    #         while cont > 0:
    #             cont, alt_s = format_string_pow(alt_s)
    #         print(alt_s)

    # for i in range(3):
    #     for j in range(6):
    #         s = "jacobian_t_({}, {}) = {};".format(i, j, j_t_num[i, j])
    #         alt_s = format_string_trig(s)
    #         cont, alt_s = format_string_pow(alt_s)
    #         while cont > 0:
    #             cont, alt_s = format_string_pow(alt_s)
    #         print(alt_s)
    # for i in range(3):
    #     for j in range(6):
    #         s = "jacobian_r_({}, {}) = {};".format(i, j, j_r_num[i, j])
    #         alt_s = format_string_trig(s)
    #         cont, alt_s = format_string_pow(alt_s)
    #         while cont > 0:
    #             cont, alt_s = format_string_pow(alt_s)
    #         print(alt_s)

    for i in range(3):
        for j in range(6):
            s = "jacobian_t_({}, {}) = {};".format(i, j, j_t[i, j])
            alt_s = format_string_trig(s)
            cont, alt_s = format_string_pow(alt_s)
            while cont > 0:
                cont, alt_s = format_string_pow(alt_s)
            print(alt_s)
    for i in range(3):
        for j in range(6):
            s = "jacobian_t_({}, {}) = {};".format(i, j, sp.simplify(j_t[i, j]))
            alt_s = format_string_trig(s)
            cont, alt_s = format_string_pow(alt_s)
            while cont > 0:
                cont, alt_s = format_string_pow(alt_s)
            print(alt_s)

    for i in range(3):
        for j in range(6):
            s = "jacobian_r_({}, {}) = {};".format(i, j, j_r[i, j])
            alt_s = format_string_trig(s)
            cont, alt_s = format_string_pow(alt_s)
            while cont > 0:
                cont, alt_s = format_string_pow(alt_s)
            print(alt_s)
    for i in range(3):
        for j in range(6):
            s = "jacobian_r_({}, {}) = {};".format(i, j, sp.simplify(j_r[i, j]))
            alt_s = format_string_trig(s)
            cont, alt_s = format_string_pow(alt_s)
            while cont > 0:
                cont, alt_s = format_string_pow(alt_s)
            print(alt_s)


#
# TransformationMatrix = Transfo_Mat()
# Jacobian = Jacobian_Mat(TransformationMatrix)
# substitution(TransformationMatrix, Jacobian)

test()
