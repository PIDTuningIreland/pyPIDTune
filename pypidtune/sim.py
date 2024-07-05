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

import io
from tkinter import messagebox
from PIL import Image, ImageTk
import ttkbootstrap as ttk
import numpy as np
import matplotlib.pyplot as plt
import common


class PIDSimulator:
    """
    Python PID Simulator Application.

    This application provides a Graphical User Interface (GUI) to simulate and visualize the response
    of a Proportional-Integral-Derivative (PID) controller using the First Order Plus Dead Time (FOPDT) model.
    """

    def __init__(self, **kwargs) -> None:
        """
        Parameters:
        -----------
        `**kwargs`: dict
            A dictionary of optional parameters for configuration.

        Keyword Arguments:
        ------------------
        - `pid_form` : str, optional

            The form of PID controller. Must be 'independent' or 'dependent'. Defaults to 'independent'.

        - `pid_time_units` : str, optional

            The time units for the PID controller. Must be 'seconds' or 'minutes'. Defaults to 'seconds'.

        - `init_model_gain` : float, optional

            Initial gain of the process model. Defaults to 2.5.

        - `init_model_tc` : float, optional

            Initial time constant of the process model. Defaults to 75.5 seconds.

        - `init_model_dt` : float, optional

            Initial dead time of the process model. Defaults to 10.5 seconds.

        - `init_model_bias` : float, optional

            Initial bias of the process model. Defaults to 13.5.
        """
        self.pid_form = kwargs.get("pid_form", "independent")
        self.pid_time_units = kwargs.get("pid_time_units", "seconds")
        if self.pid_form not in ["independent", "dependent"]:
            raise ValueError("pid_form must be 'independent' or 'dependent'")
        if self.pid_time_units not in ["minutes", "seconds"]:
            raise ValueError("pid_time_units must be 'minutes' or 'seconds'")

        try:
            init_model_gain = float(kwargs.get("init_model_gain", common.INITIAL_MODEL_GAIN))
            init_model_tc = float(kwargs.get("init_model_tc", common.INITIAL_MODEL_TC))
            init_model_dt = float(kwargs.get("init_model_dt", common.INITIAL_MODEL_DT))
            init_model_bias = float(kwargs.get("init_model_bias", common.INITIAL_MODEL_BIAS))
            init_kp = float(kwargs.get("init_kp", common.INITIAL_KP))
            init_ki = float(kwargs.get("init_ki", common.INITIAL_KI))
            init_kd = float(kwargs.get("init_kd", common.INITIAL_KD))
        except ValueError:
            raise ValueError("init_model_gain, init_model_tc, init_model_dt, init_model_bias, init_kp, init_ki, and init_kd must all be floats")

        plt.style.use("bmh")
        self.root = ttk.Window()
        self.root.title("PID Simulator - PID Tuning Ireland©")
        self.root.state("zoomed")
        # Adjust theme to suit
        style = ttk.Style(theme="yeti")
        style.theme.colors.bg = "#c0c0c0"
        style.configure(".", font=("Helvetica", 12))
        # Add frames
        self.master_frame = ttk.Frame(self.root)
        self.master_frame.pack(fill="both", expand=True)
        self.bottom_frame = ttk.Frame(self.master_frame)
        self.bottom_frame.pack(side="bottom", fill="both", expand=True)
        self.left_frame = ttk.LabelFrame(self.master_frame, text=" Controller ", bootstyle="Success")
        self.left_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)
        self.middle_frame = ttk.LabelFrame(self.master_frame, text=" Result ", bootstyle="Light")
        self.middle_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)
        self.right_frame = ttk.LabelFrame(self.master_frame, text=" Process ", bootstyle="Warning")
        self.right_frame.pack(side="right", fill="both", expand=True, padx=10, pady=10)

        # GUI Variables
        self.model_gain = ttk.DoubleVar(value=init_model_gain)
        self.model_tc = ttk.DoubleVar(value=init_model_tc)
        self.model_dt = ttk.DoubleVar(value=init_model_dt)
        self.model_bias = ttk.DoubleVar(value=init_model_bias)
        self.kp = ttk.DoubleVar(value=init_kp)
        self.ki = ttk.DoubleVar(value=init_ki)
        self.kd = ttk.DoubleVar(value=init_kd)

        # Left Frame Static Text
        ttk.Label(self.left_frame, text="PID Gains").grid(row=0, column=0, padx=10, pady=10, columnspan=2)
        if self.pid_form == "independent" and self.pid_time_units == "seconds":
            kp_unit_text = "Kp:"
            ki_unit_text = "Ki (1/sec):"
            kd_unit_text = "Kd (sec):"
        elif self.pid_form == "independent" and self.pid_time_units == "minutes":
            self.kp.set(init_kp)
            self.ki.set(init_ki * 60)
            self.kd.set(round(init_kd / 60, 4))
            kp_unit_text = "Kp:"
            ki_unit_text = "Ki (1/min):"
            kd_unit_text = "Kd (min):"
        elif self.pid_form == "dependent" and self.pid_time_units == "seconds":
            self.kp.set(init_kp)
            self.ki.set(round(init_kp / init_ki, 4))
            self.kd.set(round(init_kd / init_kp, 4))
            kp_unit_text = "Kc:"
            ki_unit_text = "Ti (sec/repeat):"
            kd_unit_text = "Td (sec):"
        elif self.pid_form == "dependent" and self.pid_time_units == "minutes":
            self.kp.set(init_kp)
            self.ki.set(round((init_kp / init_ki) / 60, 4))
            self.kd.set(round((init_kd / init_kp) / 60, 4))
            kp_unit_text = "Kc:"
            ki_unit_text = "Ti (min/repeat):"
            kd_unit_text = "Td (min):"

        ttk.Label(self.left_frame, text=kp_unit_text).grid(row=1, column=0, padx=10, pady=10)
        ttk.Label(self.left_frame, text=ki_unit_text).grid(row=2, column=0, padx=10, pady=10)
        ttk.Label(self.left_frame, text=kd_unit_text).grid(row=3, column=0, padx=10, pady=10)
        # Left Frame Entry Boxes
        ttk.Spinbox(self.left_frame, from_=-1000.00, to=1000.00, increment=0.1, textvariable=self.kp, width=15).grid(row=1, column=1, padx=10, pady=10)
        ttk.Spinbox(self.left_frame, from_=0, to=10000.00, increment=0.01, textvariable=self.ki, width=15).grid(row=2, column=1, padx=10, pady=10)
        ttk.Spinbox(self.left_frame, from_=-10000.00, to=10000.00, increment=0.01, textvariable=self.kd, width=15).grid(row=3, column=1, padx=10, pady=10)

        # Button
        button_refresh = ttk.Button(self.left_frame, text="Refresh", command=self.generate_response, bootstyle="Success")
        button_refresh.grid(row=5, column=0, columnspan=2, sticky="NESW", padx=10, pady=10)

        # Middle Frame
        self.plot_label = ttk.Label(self.middle_frame)
        self.plot_label.pack(padx=10, pady=10)

        # Right Frame Static Text
        ttk.Label(self.right_frame, text="First Order Plus Dead Time Model").grid(row=0, column=0, columnspan=2, padx=10, pady=10)
        ttk.Label(self.right_frame, text="Model Gain: ").grid(row=1, column=0, padx=10, pady=10)
        ttk.Label(self.right_frame, text="Time Constant (seconds):").grid(row=2, column=0, padx=10, pady=10)
        ttk.Label(self.right_frame, text="Dead Time (seconds):").grid(row=3, column=0, padx=10, pady=10)
        ttk.Label(self.right_frame, text="Bias:").grid(row=4, column=0, padx=10, pady=10)
        # Right Frame Entry Boxes
        ttk.Spinbox(self.right_frame, from_=-1000.00, to=1000.00, increment=0.1, textvariable=self.model_gain, width=15).grid(row=1, column=1, padx=10, pady=10)
        ttk.Spinbox(self.right_frame, from_=1.00, to=1000.00, increment=0.1, textvariable=self.model_tc, width=15).grid(row=2, column=1, padx=10, pady=10)
        ttk.Spinbox(self.right_frame, from_=1.00, to=1000.00, increment=0.1, textvariable=self.model_dt, width=15).grid(row=3, column=1, padx=10, pady=10)
        ttk.Spinbox(self.right_frame, from_=-1000.00, to=1000.00, increment=0.1, textvariable=self.model_bias, width=15).grid(row=4, column=1, padx=10, pady=10)

        # PID and Process Instantiation
        self.pid = common.PIDController()
        self.process_model = common.FOPDTModel()
        self.SP = np.zeros(0)
        self.CV = np.zeros(0)
        self.PV = np.zeros(0)
        self.pterm = np.zeros(0)
        self.iterm = np.zeros(0)
        self.dterm = np.zeros(0)
        self.itae = 0
        # Create plot buffer and generate blank plot
        self.image_buffer = io.BytesIO()
        self.generate_plot()

    def generate_response(self) -> None:
        """
        Generate the response of the PID controller and the FOPDT model.

        This method calculates the control values, process variable, and individual controller
        components based on the given model parameters and PID gains.
        It also calculates the Integral Time Weighted Average of the Error (ITAE) as a performance measure.
        """
        try:
            model_params = (self.model_gain.get(), self.model_tc.get(), self.model_dt.get(), self.model_bias.get())
            sim_gains = common.convert_gains_for_sim(gains=(self.kp.get(), self.ki.get(), self.kd.get()), pid_form=self.pid_form, pid_time_units=self.pid_time_units)
            plot_data = common.calculate_pid_response(model_params=model_params, tune_values=sim_gains)
            # Create arrays to store the simulation results
            self.SP = plot_data[0]
            self.CV = plot_data[1]
            self.PV = plot_data[2]
            self.pterm = plot_data[3]
            self.iterm = plot_data[4]
            self.dterm = plot_data[5]
            self.itae = plot_data[6]
            # Update the plot
            self.generate_plot()

        except Exception as e:
            messagebox.showerror("An Error Occurred", "Check Configuration: " + str(e))

    def generate_plot(self) -> None:
        """
        Generate the plot for the response of the PID controller and the FOPDT model.
        """
        plt.figure()
        plt.subplot(2, 1, 1)
        plt.plot(self.SP, color="goldenrod", linewidth=2, label="SP")
        plt.plot(self.CV, color="darkgreen", linewidth=2, label="CV")
        plt.plot(self.PV, color="blue", linewidth=2, label="PV")
        plt.ylabel("Value")
        plt.suptitle(f"ITAE: {round(self.itae, 2)}")
        if self.pid_form == "independent":
            title = f"Kp: {self.kp.get()}  Ki: {self.ki.get()}  Kd: {self.kd.get()}"
        else:
            title = f"Kc: {self.kp.get()}  Ti: {self.ki.get()}  Td: {self.kd.get()}"
        plt.title(title, fontsize=10)
        plt.legend(loc="best")

        # Individual components
        plt.subplot(2, 1, 2)
        plt.plot(self.pterm, color="lime", linewidth=2, label="P Term")
        plt.plot(self.iterm, color="orange", linewidth=2, label="I Term")
        plt.plot(self.dterm, color="purple", linewidth=2, label="D Term")
        plt.xlabel("Time [seconds]")
        plt.ylabel("Value")
        plt.legend(loc="best")
        plt.grid(True)
        plt.savefig(self.image_buffer, format="png")
        plt.close()

        # Convert plot to tkinter image
        img = Image.open(self.image_buffer)
        photo_img = ImageTk.PhotoImage(img)
        # Delete the existing plot
        self.plot_label.configure(image="")
        self.plot_label.image = ""
        # Add the new plot
        self.plot_label.configure(image=photo_img)
        self.plot_label.image = photo_img
        # Rewind the tape
        self.image_buffer.seek(0)


if __name__ == "__main__":
    sim_app = PIDSimulator()
    sim_app.root.mainloop()
