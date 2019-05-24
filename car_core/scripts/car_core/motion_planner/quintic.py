# This file is licensed under MIT license.
# See the LICENSE file in the project root for more information.

import numpy as np
import numpy.polynomial.polynomial as npoly

# [DEPRICATED]
# Numerical approach to calc polynom coefficients
def calc_coefs_numeric(s1, s2, T):
    A = np.array([[   0,       0,       0,      0,    0, 1],
                  [   0,       0,       0,      0,    1, 0],
                  [   0,       0,       0,      2,    0, 0],
                  [   T**5,    T**4,    T**3,   T**2, T, 1],
                  [ 5*T**4,  4*T**3,  3*T**2, 2*T,    1, 0],
                  [20*T**3, 12*T**2,  6*T,      2,    0, 0]])
    B = np.array([s1[0], s1[1], s1[2], s2[0], s2[1], s2[2]])
    return np.linalg.solve(A, B)

# Symbol approach to calc polynom coefficients
def calc_coefs(s0, s1, T):
    return [(T**2*(-s0[2] + s1[2]) - 6*T*(s0[1] + s1[1]) - 12*s0[0] + 12*s1[0])/(2*T**5),
            (T**2*(3*s0[2] - 2*s1[2])/2 + T*(8*s0[1] + 7*s1[1]) + 15*s0[0] - 15*s1[0])/T**4,
            (T**2*(-3*s0[2] + s1[2]) - 4*T*(3*s0[1] + 2*s1[1]) - 20*s0[0] + 20*s1[0])/(2*T**3),
            s0[2]/2,
            s0[1],
            s0[0]]

# [DEPRICATED]
# Calc single point of the polynom interpolation
def calc_quintic(coefs, x):
    return np.sum(coefs * np.power(x, np.linspace(5,0,6)))
def calc_dquintic(coefs, x):
    return np.sum(coefs[:-1] * np.power(x, np.linspace(4,0,5)) * np.array([5,4, 3, 2, 1]))
def calc_ddquintic(coefs, x):
    return np.sum(coefs[:-2] * np.power(x, np.linspace(3,0,4)) * np.array([20, 12, 6, 2]))

# Interpolate quntic polynom y(x) and it's derivatives dy/dx, d2y/dx^2 for given x-values
def interpolate(coefs, x):
    y =   npoly.polyval(x, coefs[::-1])
    dy =  npoly.polyval(x, [coefs[4], 2*coefs[3], 3*coefs[2], 4*coefs[1], 5*coefs[0]])
    ddy = npoly.polyval(x, [2*coefs[3], 6*coefs[2], 12*coefs[1], 20*coefs[0]])
    return y, dy, ddy

# Interpolate jerk (d3y/dx^3)
#def inerpolate_jerk(coefs, x):
#    return npoly.polyval(x, [6*coefs[2], 24*coefs[1], 60*coefs[0]])