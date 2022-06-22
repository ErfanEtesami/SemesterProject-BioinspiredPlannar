import numpy as np


def gen_bezier_poly(beta):
    """
    Generate the coefficients of the Bezier polynomial.
    """
    n = len(beta) - 1
    coeffs = np.zeros(n+1)
    for i in range(n+1):
        temp = 0
        for j in range(i+1):
            temp = temp + (((-1)**(i+j)) * beta[j]) / (np.math.factorial(j) * np.math.factorial(i-j))
        coeffs[i] = (np.math.factorial(n) / np.math.factorial(n-i)) * temp
    return coeffs


def gen_cnt(t_st, t_alpha, t_st_array, beta_1, beta_2):
    """
    Generate contact force profiles.
    """
    coeffs_1 = gen_bezier_poly(beta_1)
    coeffs_2 = gen_bezier_poly(beta_2)
    force = np.zeros(len(t_st_array))
    for i in range(len(t_st_array)):
        if t_st_array[i] <= t_alpha:
            s = t_st_array[i] / t_alpha
            for j in range(len(coeffs_1)):
                force[i] = force[i] + coeffs_1[j] * (s**j)
        elif t_st_array[i] > t_alpha:
            s = (t_st_array[i]-t_alpha) / (t_st-t_alpha)
            for j in range(len(coeffs_2)):
                force[i] = force[i] + coeffs_2[j] * (s**j)
    return force


def gen_afb(t_st, t_alpha_1, t_alpha_2, t_st_array, beta_1, beta_2):
    """
    Generate the activation feedback.
    """
    coeffs_1 = gen_bezier_poly(beta_1)
    coeffs_2 = gen_bezier_poly(beta_2)
    afb = np.zeros(len(t_st_array))
    for i in range(len(t_st_array)):
        if t_st_array[i] <= t_alpha_1:
            s = t_st_array[i]/t_alpha_1
            for j in range(len(coeffs_1)):
                afb[i] = afb[i] + coeffs_1[j] * (s**j)
        elif t_alpha_1 < t_st_array[i] <= t_alpha_2:
            afb[i] = 1
        elif t_st_array[i] > t_alpha_2:
            s = (t_st_array[i] - t_alpha_2) / (t_st - t_alpha_2)
            for j in range(len(coeffs_2)):
                afb[i] = afb[i] + coeffs_2[j] * (s**j)
    return afb


def gen_traj_sw(t_sw, t_sw_array, beta):
    """
    Generate the desired original swing trajectory.
    """
    coeffs = gen_bezier_poly(beta)
    traj = np.zeros(len(t_sw_array))
    for i in range(len(t_sw_array)):
        s = t_sw_array[i] / t_sw
        for j in range(len(coeffs)):
            traj[i] = traj[i] + coeffs[j] * (s**j)
    return traj


def gen_s_sw(t_sw, t_sw_array, beta):
    """
    Generate the s_sw parameter in the form of a Bezier polynomial.
    """
    coeffs = gen_bezier_poly(beta)
    s_sw = np.zeros(len(t_sw_array))
    for i in range(len(t_sw_array)):
        s = t_sw_array[i] / t_sw
        for j in range(len(coeffs)):
            s_sw[i] = s_sw[i] + coeffs[j] * (s**j)
    return s_sw


def gen_traj_sw_mdf(s_sw, beta):
    """
    Generate the modified swing trajectory.
    """
    coeffs = gen_bezier_poly(beta)
    traj_mdf = np.zeros(len(s_sw))
    for i in range(len(s_sw)):
        for j in range(len(coeffs)):
            traj_mdf[i] = traj_mdf[i] + coeffs[j] * (s_sw[i]**j)
    return traj_mdf


def gen_traj_sw_crr(s_sw, pos_f, traj_sw_mdf):
    """
    Generate the corrected swing trajectory.
    """
    s_crr = np.minimum(2*s_sw, 1)
    alpha = pos_f-traj_sw_mdf[0]
    beta = [alpha, alpha, 0, 0]
    coeffs = gen_bezier_poly(beta)
    h_crr = np.zeros(len(s_crr))
    for i in range(len(s_crr)):
        for j in range(len(coeffs)):
            h_crr[i] = h_crr[i] + coeffs[j] * (s_crr[i]**j)
    traj_sw_crr = traj_sw_mdf + h_crr
    return traj_sw_crr
