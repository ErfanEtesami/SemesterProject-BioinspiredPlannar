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
