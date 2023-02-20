from simulator import Arm, Simulator, GUI
import matplotlib.pyplot as plt
from tkinter import Tk, HORIZONTAL, Scale, Label, IntVar

arm = Arm(105, 90, 65, 105, opt=Arm.minimum_change, implementation="t")

g = GUI(arm, True, [8, 5, 4, 2, 1, 0])
g.openGUI()
