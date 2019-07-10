import numpy as np
import math

# all operands should be numpy arrays


def translate(point, offset):
    result = point + offset
    return result


def rotate_z(point, angle, origin=None):
    origin = origin if origin is not None else np.array([[0, 0, 0]])

    c, s = np.cos(angle), np.sin(angle)
    rot = np.array([(c, -s, 0),
                    (s, c, 0),
                    (0, 0, 1)])  # 3d rotation matrix for z rotation
    result = point - origin
    result = np.matmul(result, rot)

    result += origin
    return result


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    points = np.array([[1, 1, 0], [2, 1, 1]])
    orig = np.array([[1, 1, 1]])

    w = rotate_z(points, np.radians(90), orig)
    v = points

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(v[:, 0], v[:, 1], v[:, 2])

    #fig2 = plt.figure()
    #ax2 = fig2.add_subplot(111, projection='3d')
    ax.plot(w[:, 0], w[:, 1], w[:, 2])

    plt.show()

