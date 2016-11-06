import numpy as np
from sympy.solvers import solve
from sympy import Symbol


def normalize_sensor_readings(readings_arr):
    arr = []

    for idx, r in enumerate(readings_arr):
        x = Symbol('x')

        if idx == 0:
            arr.append(solve(-7.3804 * x ** 3 + 109.3405 * x ** 2 - 540.5984 * x + 1028.6119 - r, x)[0])

        elif idx == 1:
            arr.append(solve(-17.6896 * x ** 3 + 195.52 * x ** 2 - 721.418 * x + 1014.84 - r, x)[0])

        elif idx == 2:
            arr.append(solve(-3.075207398 * x ** 5 + 48.26268901 * x ** 4 - 294.5528620 * x ** 3 + 885.3472322 * x ** 2 - 1361.6370243 * x + 1020 - r, x)[0])

        elif idx == 3:
            arr.append(solve(-0.560244443 * x ** 5 + 11.41072057 * x ** 4 - 94.7792302 * x ** 3 + 405.5557011 * x ** 2 - 920.3395907 * x + 1020 - r, x)[0])

        elif idx == 4:
            arr.append(solve(-15.9683156122 * x ** 3 + 178.782916474 * x ** 2 - 675.967600511 * x + 1014.962391210 - r, x)[0])

        elif idx == 5:
            arr.append(solve(-15.631530459 * x ** 3 + 178.22568971 * x ** 2 - 684.61057171 * x + 1016.46983217 - r, x)[0])

        elif idx == 6:
            arr.append(solve(-12.5520697167 * x ** 3 + 148.793464052 * x ** 2 - 608.975287892 * x + 1018.118394024 - r, x)[0])

        elif idx == 7:
            arr.append(
                solve(-18.0936819173 * x ** 3 + 198.762278245 * x ** 2 - 722.771677561 * x + 1015.180205416 - r, x)[0])

        else:
            raise Exception("something went wrong idx=" + str(idx))

    def to_positive_float(n):
        try:
            n = float(n)
            return 0 if n < 0 else n

        except TypeError:
            raise TypeError('Encountered an imaginary number')

    return list(map(to_positive_float, arr))


def safe_arctan(x, y):
    if x == 0:
        if y > 0:
            return 3 / 2 * np.pi
        else:
            return np.pi / 2
    ang = np.arctan(y / x)
    if ang < 0:
        ang += 2 * np.pi
    if y < 0:
        ang += np.pi
    return ang

