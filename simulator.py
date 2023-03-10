import time
from numpy.linalg import norm
import numpy as np
from tkinter import Tk, HORIZONTAL, Scale, Label, IntVar
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from math import pi, cos, sin, asin, acos, atan2, inf, sqrt, ceil, floor
import matplotlib

import Cuplib as Cup
matplotlib.use("TkAgg")  # Mac requires matplotlib to explicitly use tkinter

half_pi = pi / 2


class Arm:
    steps = 10
    step_length = 1 / steps
    default_position = np.array([90, 90, 90, 90, 90, 45], np.float64)

    @staticmethod
    def default_optimization(d1, d2, d3, p):
        return norm([d1, d2, d3], ord=1)

    @staticmethod
    def minimum_change(d1, d2, d3, p):
        return norm(90 - (np.rad2deg(np.array([d1, d2, d3])) - p[:3]), ord=2)

    def __init__(self, r1, r2, r3, r4, x=0, y=150, z=150, opt=default_optimization, implementation="ax"):
        """
          :param r1: Length of the first segment
          :param r2: Length of the second segment
          :param r3: Length of the third segment
          :param r4: Length of the claws
          :param opt: The optimization function.
          """
        self.r1 = r1
        self.r2 = r2
        self.r3 = r3
        self.r4 = r4
        self.x, self.y, self.z = x, y, z
        self.len = r1 + r2 + r3
        self.position = Arm.default_position.copy()
        self.rad_pos = np.deg2rad(self.position)

        self.opt = opt

        # Stands for Tom: Hanzhi Zhou's implementation by analytically solve the inequality using algebraic method
        if implementation == "t":
            self.solve_three = self._t_solve_angle

        # Stands for Alex: Yuhao Zhou's implementation by analytically solve the inequality using angles
        elif implementation == "ax":
            self.solve_three = self._ax_solve_angle

        # Stands for Tom Traversal: Hanzhi Zhou's original implementation by traversing all possible values of m
        elif implementation == "tt":
            self.solve_three = self._t_solve_angle
            self._get_m_range = self._get_m_range_traverse

        else:
            raise ValueError('implementation must be one of "t", "ax" and "tt"')

    def get_radians(self, x, y, z):
        """
        :param x:
        :param y:
        :param z:
        :return: a list of radians
        """
        distance = norm((x, y, z), ord=2)
        if distance > self.len:
            raise Exception("Too far to reach!")

        ds = np.zeros(4, dtype=np.float64)
        ds[0] = atan2(y, x)
        # if not pi * 0.75 < d0 < -pi * 0.25:
        #     raise Exception("Out of range!")

        if not self.solve_three(norm((x, y), ord=2), z, ds):
            raise Exception("!!")
        return ds

    def calc_ds(self, a, m, b, n, d1, arr):
        """
        calculate d2 and d3
        """
        temp = (a - m) ** 2 + (b - n) ** 2
        d2 = half_pi - d1 - acos((self.r2 ** 2 + temp - self.r3 ** 2)
                                 / (2 * self.r2 * sqrt(temp))) - atan2((b - n), (a - m))
        if not -half_pi <= d2 <= half_pi:
            return
        d3 = pi - acos((self.r2 ** 2 + self.r3 ** 2 - temp) / (2 * self.r2 * self.r3))
        if not -half_pi <= d3 <= half_pi:
            return
        arr[1] = d2
        arr[2] = d3
        return self.opt(d1, d2, d3, self.position)

    def _t_solve_angle(self, a, b, arr):
        """
        :param a: x coordinate in the plane
        :param b: y coordinate in the plane
        :return: a list of radians
        """

        s = inf
        temp_arr = np.zeros(3, dtype=np.float64)
        for m in self._get_m_range(a, b):
            d1 = asin(m / self.r1)
            temp_arr[0] = d1
            opt_val = self.calc_ds(a, m, b, sqrt(self.r1 ** 2 - m ** 2), d1, temp_arr)
            if opt_val is not None and opt_val < s:
                s = opt_val
                arr[1:] = temp_arr
        return s != inf

    def _get_m_range_traverse(self, a, b):
        rg = []
        c1 = sqrt(self.r2 ** 2 + self.r3 ** 2)
        c2 = (self.r2 + self.r3) ** 2
        for i in range(-self.r1 * Arm.steps, self.r1 * Arm.steps):
            m = i / Arm.steps
            v = (a - m) ** 2 + (b - sqrt(self.r1 ** 2 - m ** 2)) ** 2
            if c1 < v < c2:
                rg.append(m)
        return rg

    def _get_m_range(self, a, b):
        l1 = self.r1
        l2 = self.r2
        l3 = self.r3

        A = l2 ** 2 + l3 ** 2 - l1 ** 2 - a ** 2 - b ** 2

        temp = (a ** 2 + b ** 2) * 2
        d1 = 2 * temp * l1 ** 2 - A ** 2

        if d1 > 0:
            _sq_d1 = sqrt(d1)
            m1 = int((-A * a - b * _sq_d1) / temp * Arm.steps)
            m2 = int((-A * a + b * _sq_d1) / temp * Arm.steps)
        else:
            m1 = 0
            m2 = 0

        B = -A - 2 * l2 * l3
        d2 = 2 * temp * l1 ** 2 - B ** 2

        u = int(B / (2 * a) * Arm.steps)

        if d2 > 0:
            _sq_d2 = sqrt(d2)
            m3 = int((B * a - b * _sq_d2) / temp * Arm.steps)
            m4 = int((B * a + b * _sq_d2) / temp * Arm.steps)
        else:
            m3 = 0
            m4 = 0

        if b != 0:
            return np.intersect1d(
                np.union1d(
                    np.arange(-l1 * Arm.steps, m1, dtype=np.int32), np.arange(m2,
                                                                              l1 * Arm.steps, dtype=np.int32)
                ),
                np.union1d(
                    np.arange(m3, m4, dtype=np.int32), np.arange(u, l1 * Arm.steps, dtype=np.int32)
                ),
                assume_unique=True
            ) / Arm.steps
        else:
            return np.arange(u + Arm.step_length, -A / (2 * a), Arm.step_length)

    # For Alex's implementation
    def _ax_solve_angle(self, a, b, arr):
        s = inf
        temp_arr = np.zeros(3, dtype=np.float64)
        for k in self._get_angle_range(a, b):
            d1 = np.deg2rad(k)
            temp_arr[0] = d1
            opt_val = self.calc_ds(a, self.r1 * sin(d1), b, self.r1 * cos(d1), d1, temp_arr)
            if opt_val is not None and opt_val < s:
                s = opt_val
                arr[1:] = temp_arr
        return s != inf

    # For Alex's implementation
    def _get_angle_range(self, a, b):
        theta_range = []
        k = sqrt(a**2 + b**2)  # distance
        phi = atan2(b, a)

        lim_min = self.r2 ** 2 + self.r3 ** 2
        lim_max = (self.r2 + self.r3) ** 2
        m_min = (k ** 2 + self.r1 ** 2 - lim_max) / (2 * k * self.r1)
        m_max = (k ** 2 + self.r1 ** 2 - lim_min) / (2 * k * self.r1)

        if m_min < -1:  # Will this be a case?
            m_min = -1
        angle_1 = asin(m_min)

        angle_3 = 0
        angle_4 = 0
        if m_max > 1:
            print(m_max)
            if pi - angle_1 - phi > pi / 2:  # Will this be a case?
                angle_2 = pi / 2
            else:
                angle_2 = pi - angle_1
        else:
            angle_2 = asin(m_max)

        if pi / 2 + phi > pi - angle_1:
            angle_3 = pi - angle_2
            angle_4 = pi - angle_1
        elif pi - angle_2 <= pi / 2 + phi <= pi - angle_1:
            angle_3 = pi - angle_2
            angle_4 = pi / 2 + phi

        angle_1 = angle_1 - phi
        angle_2 = angle_2 - phi

        c = np.rad2deg(np.array([angle_1, angle_2]))
        if angle_3 != 0 and angle_4 != 0:
            c = np.concatenate((c, np.rad2deg(np.array([angle_3, angle_4]))), axis=0)
            for i in range(ceil(Arm.steps * c[0]), floor(Arm.steps * c[1] + 1)):
                theta_range.append(i / Arm.steps)
            for i in range(ceil(Arm.steps * c[2]), floor(Arm.steps * c[3] + 1)):
                theta_range.append(i / Arm.steps)
        else:
            for i in range(ceil(Arm.steps * c[0]), floor(Arm.steps * c[1] + 1)):
                theta_range.append(i / Arm.steps)

        return theta_range

    # convert degrees to servo angles
    @staticmethod
    def _cov_degs(degs):
        # degs[0] += 45
        degs[1] = 90 - degs[1]
        degs[2] = 90 - degs[2]
        degs[3] = 90 - degs[3]
        return degs

    @staticmethod
    def _cov_rads(rads):
        # rads[0] += pi / 4
        rads[1] = half_pi - rads[1]
        rads[2] = half_pi - rads[2]
        rads[3] = half_pi - rads[3]
        return rads

    # Update the joint angles so that the arm can reach (x, y, z)
    def goto(self, x=None, y=None, z=None):
        if x is None:
            x = self.x
        if y is None:
            y = self.y
        if z is None:
            z = self.z

        self.rad_pos = self.get_radians(x, y, z)
        self.position = np.concatenate(
            (self._cov_degs(np.rad2deg(self.rad_pos)), self.position[4:]), axis=0
        )
        self.x, self.y, self.z = x, y, z

    # get coordinates of each joint in three dimensional space
    def get_coordinates(self):
        d0 = self.rad_pos[0]
        d1 = pi / 2 - self.rad_pos[1]
        x1, y1 = cos(d1) * self.r1 * cos(d0), cos(d1) * self.r1 * sin(d0)
        z1 = sin(d1) * self.r1
        d2 = d1 - self.rad_pos[2]
        x2, y2 = cos(d2) * self.r2 * cos(d0) + x1, cos(d2) * self.r2 * sin(d0) + y1
        z2 = z1 + sin(d2) * self.r2
        r3 = self.r3
        d3 = d2 - self.rad_pos[3]
        x3, y3 = cos(d3) * r3 * cos(d0) + x2, cos(d3) * r3 * sin(d0) + y2
        z3 = z2 + sin(d3) * r3

        return np.array([0, x1, x2, x3, 0, y1, y2, y3, 0, z1, z2, z3], dtype=np.float64).reshape((3, 4))

    def set_position(self, idx, rot):
        self.position[idx] = rot
        if 0 <= idx < 4:
            self.rad_pos = np.deg2rad(self.position)
            self._cov_rads(self.rad_pos)
            points = self.get_coordinates()
            self.x = points[0, -1]
            self.y = points[1, -1]
            self.z = points[2, -1]

    def open_claws(self):
        self.position[5] = 0

    def close_claws(self):
        self.position[5] = 79


# from https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula


def vec_rot(vec, axis, theta):
    return vec * cos(theta) + np.cross(axis, vec) * sin(theta) + axis * np.dot(axis, vec) * (1 - cos(theta))


class Simulator:

    def __init__(self, arm: Arm, ax: Axes3D):
        self.arm = arm
        self.ax = ax
        self.update()

    def update(self, xmin=-50, xmax=100, ymin=-50, ymax=100, zmin=-50, zmax=200):
        _points = self.arm.get_coordinates()
        points = _points.T[1:]
        last_line = points[2] - points[1]
        last_line /= norm(last_line)  # unit vector
        normal = np.cross(points[0] - points[1], points[0] - points[2])

        rot_axis = np.cross(normal, last_line)
        rot_axis /= norm(rot_axis)

        # claws rotation (open or close): axis of rotation is the normal plane containing the claws
        rot = np.deg2rad(self.arm.position[5]) / 2
        temp1 = vec_rot(last_line * self.arm.r4, rot_axis, rot)
        temp2 = vec_rot(last_line * self.arm.r4, rot_axis, -rot)

        # fifth servo rotation: axis of rotation is the last piece of arm
        rot = np.deg2rad(self.arm.position[4])
        claw_vec_rot1 = vec_rot(temp1, last_line, rot)
        claw_vec_rot2 = vec_rot(temp2, last_line, rot)

        self.ax.clear()
        self.ax.set_xlabel('X / mm')
        self.ax.set_xlim(xmin, xmax)
        self.ax.set_ylabel('Y / mm')
        self.ax.set_ylim(ymin, ymax)
        self.ax.set_zlabel('Z / mm')
        self.ax.set_zlim(zmin, zmax)
        self.ax.plot(_points[0], _points[1], _points[2], linewidth=2.5,
                     marker='*', markersize=8, markerfacecolor='y')
        self.ax.scatter(_points[0, -1], _points[1, -1], _points[2, -1], c='r', s=80, marker='o')

        claw_points = np.array([points[2], points[2] + claw_vec_rot1]).transpose()
        self.ax.plot(claw_points[0], claw_points[1], claw_points[2],
                     linewidth=2.5, marker='*', markersize=8, markerfacecolor='y', c="g")
        claw_points = np.array([points[2], points[2] + claw_vec_rot2]).transpose()
        self.ax.plot(claw_points[0], claw_points[1], claw_points[2],
                     linewidth=2.5, marker='*', markersize=8, markerfacecolor='y', c="g")

class GUI():
    def __init__(self, arm, connected=False, cupList=[]):
        Cup.Setpause(cupList)
        self.connected = connected
        self.cupList=cupList
        self.arm = arm
        self.LastCup = [90, 90, 90, 90, 90, 90]
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(projection='3d')
        self.simulator = Simulator(self.arm, self.ax)
        self.xmin = -150
        self.xmax = 150
        self.ymin = -150
        self.ymax = 150
        self.zmin = 0
        self.zmax = 150
        self.root = Tk()
        self.root.title('Robotic Arm Control Simulation')
        self.root.configure(background='white')
        self.int_vars = [IntVar(self.root) for i in range(9)]
        self.texts = ["x (mm): ", "y (mm): ", "z (mm): ", "0 (deg): ", "1 (deg): ",
                      "2 (deg): ", "3 (deg): ", "claw (deg): ", "open (deg): "]
        for i, txt in enumerate(self.texts):
            self.label = Label(self.root, text=txt, font=("Courier", 14))
            self.label.configure(background="white")
            self.label.grid(row=i)

        self.sx = Scale(self.root, from_=self.xmin, to_=self.xmax, orient=HORIZONTAL,
                   length=600, command=lambda t: self.update(0, t), variable=self.int_vars[0])
        self.sy = Scale(self.root, from_=self.ymin, to_=self.ymax, orient=HORIZONTAL,
                   length=600, command=lambda t: self.update(1, t), variable=self.int_vars[1])
        self.sz = Scale(self.root, from_=self.zmin, to_=self.zmax, orient=HORIZONTAL,
                   length=600, command=lambda t: self.update(2, t), variable=self.int_vars[2])
        
        self.servos = []
        for i in range(6):
            self.servo = (
                lambda i:
                Scale(self.root, from_=0, to_=180, orient=HORIZONTAL, length=600,
                      command=lambda t: self.update(3 + i, t),    variable=self.int_vars[3 + i])
            )(i)  # closure
            self.servo.configure(background="white")
            self.servos.append(self.servo)
            self.servo.set(self.arm.position[i])
            self.servo.grid(row=3 + i, column=1)

    def update(self, n, t):
        if n == 0:
            self.arm.goto(x=self.int_vars[0].get())
            self.update_servo_scale()
        elif n == 1:
            self.arm.goto(y=self.int_vars[1].get())
            self.update_servo_scale()
        elif n == 2:
            self.arm.goto(z=self.int_vars[2].get())
            self.update_servo_scale()
        elif 3 <= n < 9:
            self.arm.set_position(n - 3, self.int_vars[n].get())
            if n < 7:  # 7 and 8 are the claws, no need to self.update
                self.update_xyz_scale()

        self.simulator.update(self.xmin, self.xmax, self.ymin, self.ymax, self.zmin, self.zmax)

    def update_servo_scale(self):
        for i, servo in enumerate(self.servos):
            self.int_vars[3 + i].set(self.arm.position[i])
            #if self.connected:
                #for i in range(len(self.cupList)):
                    #Cup.tMove(self.cupList[i], self.LastCup[i], int(self.arm.position[i]), 0.01)
                    #self.LastCup[i] = int(self.arm.position[i])

    def update_xyz_scale(self):
        self.int_vars[0].set(self.arm.x)
        self.int_vars[1].set(self.arm.y)
        self.int_vars[2].set(self.arm.z)

    def openGUI(self):
        self.sx.set(100)
        self.sy.set(100)
        self.sz.set(100)
        self.sx.configure(background="white")
        self.sy.configure(background="white")
        self.sz.configure(background="white")
        self.sx.grid(row=0, column=1)
        self.sy.grid(row=1, column=1)
        self.sz.grid(row=2, column=1)

        plt.show()
        self.root.mainloop()
