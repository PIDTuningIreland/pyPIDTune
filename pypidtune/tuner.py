"""
   
Updated and maintained by pidtuningireland@gmail.com
Copyright 2024 pidtuningireland

Licensed under the MIT License;
you may not use this file except in compliance with the License.

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
   
"""

import os
import threading
from tkinter import filedialog as fd
import numpy as np
from scipy.optimize import minimize
import ttkbootstrap as ttk
import matplotlib.pyplot as plt
import common


class PIDTuner:
    """
    PIDTuner is responsible for generating a First Order Plus Dead Time model of the process and
    providing PID gains based on that model.

    """

    def __init__(self):
        plt.style.use("bmh")
        self.root = ttk.Window()
        self.root.title("PID Tuner - PID Tuning Ireland©")
        # Adjust theme to suit
        style = ttk.Style(theme="yeti")
        style.theme.colors.bg = "#c0c0c0"
        style.configure("TRadiobutton", background="#c0c0c0")

        self.screen_width = self.root.winfo_screenwidth()
        self.screen_height = self.root.winfo_screenheight()
        self.offset = 7
        self.toolbar = 73
        self.root.resizable(True, True)
        self.root.geometry(f"{int(self.screen_width/2)}x{self.screen_height-self.toolbar}+{-self.offset}+0")
        ttk.Label(self.root, text="PID Tuner Not Available Yet...").grid()


if __name__ == "__main__":
    tuner_app = PIDTuner()
    tuner_app.root.mainloop()
