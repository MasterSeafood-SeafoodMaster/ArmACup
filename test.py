from simulator import Arm, Simulator, GUI
import matplotlib.pyplot as plt
from tkinter import Tk, HORIZONTAL, Scale, Label, IntVar

arm = Arm(105, 90, 65, 105, opt=Arm.minimum_change, implementation="t")

g = GUI(arm)
g.openGUI()