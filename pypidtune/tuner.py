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

    def __init__(self, **kwargs):
        """
        Parameters:
        -----------
        `**kwargs`: dict
            A dictionary of optional parameters for configuration.

        Keyword Arguments:
        ------------------
        - `pid_type` : str, optional

            The type of PID controller. Must be 'pi' or 'pid'. Defaults to 'pid'.

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

        - `default_csv_sep` : str, optional

            Separator for CSV file. Defaults to ";".
        """
        # Initialize instance attributes based on provided or default values
        self.pid_type = kwargs.get("pid_type", "pid")
        self.pid_form = kwargs.get("pid_form", "independent")
        self.pid_time_units = kwargs.get("pid_time_units", "seconds")
        if self.pid_type not in ["pi", "pid"]:
            raise ValueError("pid_type must be 'pi' or 'pid'")
        if self.pid_form not in ["independent", "dependent"]:
            raise ValueError("pid_form must be 'independent' or 'dependent'")
        if self.pid_time_units not in ["minutes", "seconds"]:
            raise ValueError("pid_time_units must be 'minutes' or 'seconds'")
        try:
            self.init_model_gain = float(kwargs.get("init_model_gain", common.INITIAL_MODEL_GAIN))
            self.init_model_tc = float(kwargs.get("init_model_tc", common.INITIAL_MODEL_TC))
            self.init_model_dt = float(kwargs.get("init_model_dt", common.INITIAL_MODEL_DT))
            self.init_model_bias = float(kwargs.get("init_model_bias", common.INITIAL_MODEL_BIAS))
        except ValueError:
            raise ValueError("init_model_gain, init_model_tc, init_model_dt and init_model_bias must all be floats")

        self.default_csv_sep = kwargs.get("default_csv_sep", ";")

        self._build_gui()
        self._reset_values_on_gui()
        self._load_empty_plot()

    def _build_gui(self):
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

        self.base_frame = ttk.Frame(self.root)
        self.base_frame.pack(expand=True, fill=ttk.BOTH)

        self.upper_frame = ttk.Frame(self.base_frame)
        self.upper_frame.pack(expand=False, side=ttk.TOP, fill=ttk.BOTH)

        self.middle_frame = ttk.Frame(self.base_frame)
        self.middle_frame.pack(expand=False, side=ttk.TOP, fill=ttk.BOTH, pady=10)

        self.lower_frame = ttk.Frame(self.base_frame)
        self.lower_frame.pack(expand=False, side=ttk.BOTTOM, fill=ttk.BOTH)

        self.button_frame = ttk.Frame(self.upper_frame)
        self.button_frame.pack(side=ttk.LEFT, fill=ttk.BOTH)

        self.file_frame = ttk.Frame(self.upper_frame)
        self.file_frame.pack(side=ttk.LEFT, fill=ttk.BOTH, pady=5)

        self.model_frame = ttk.Frame(self.middle_frame)
        self.model_frame.pack(expand=True, side=ttk.LEFT, fill=ttk.BOTH)

        self.tune_frame = ttk.Frame(self.middle_frame)
        self.tune_frame.pack(expand=True, side=ttk.RIGHT, fill=ttk.BOTH)

        self.file_name = ttk.StringVar()
        self.model_gain = ttk.StringVar(value="...")
        self.model_tc = ttk.StringVar(value="...")
        self.model_dt = ttk.StringVar(value="...")
        self.model_bias = ttk.StringVar(value="...")
        self.kp_unit = ttk.StringVar()
        self.ki_unit = ttk.StringVar()
        self.kd_unit = ttk.StringVar()
        self.kp = ttk.StringVar()
        self.ki = ttk.StringVar()
        self.kd = ttk.StringVar()
        self.gains = {}
        self.original_sample_rate = 100
        self.sample_rate = ttk.IntVar(value=self.original_sample_rate)
        self.csv_sep = ttk.StringVar(value=self.default_csv_sep)
        self.gui_status = ttk.StringVar()

        self.open_file_button = ttk.Button(self.button_frame, text="Open CSV File", command=self.open_file)
        self.open_file_button.grid(row=0, column=0, padx=10, pady=10, sticky=ttk.NSEW)

        self.generate_model_button = ttk.Button(self.button_frame, text="Generate Model", command=self.solve_for_minimum_error)
        self.generate_model_button.grid(row=1, column=0, padx=10, pady=10, sticky=ttk.NSEW)
        self.generate_model_button.configure(state=ttk.DISABLED)

        self.sim_pid_button = ttk.Button(self.button_frame, text="Simulate PID Response", command=self.sim_pid)
        self.sim_pid_button.grid(row=3, column=0, padx=10, pady=10, sticky=ttk.NSEW)
        self.sim_pid_button.configure(state=ttk.DISABLED)

        ttk.Label(self.file_frame, text="Filename:").grid(row=0, column=0, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.file_frame, textvariable=self.file_name).grid(row=0, column=1, columnspan=3, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.file_frame, text="CSV Separator:").grid(row=1, column=0, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.file_frame, textvariable=self.csv_sep).grid(row=1, column=1, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.file_frame, text="Data Sample Interval:").grid(row=2, column=0, padx=10, pady=10, sticky=ttk.W)
        self.sample_rate_entry = ttk.Entry(self.file_frame, textvariable=self.sample_rate, width=6)
        self.sample_rate_entry.grid(row=2, column=1, padx=10, pady=10, sticky=ttk.W)
        self.sample_rate_entry.bind("<Return>", self.change_sample_rate)
        self.sample_rate_entry.bind("<FocusOut>", self.change_sample_rate)
        ttk.Label(self.file_frame, text="mS").grid(row=2, column=2, padx=10, pady=10, sticky=ttk.W)

        ttk.Label(self.model_frame, text="Model Gain:").grid(row=1, column=1, columnspan=2, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.model_frame, text="Time Constant (sec):").grid(row=2, column=1, columnspan=1, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.model_frame, text="Dead Time (sec):").grid(row=3, column=1, columnspan=1, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.model_frame, text="Ambient/Bias:").grid(row=4, column=1, columnspan=1, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.model_frame, textvariable=self.model_gain, width=15).grid(row=1, column=2, padx=10, pady=10, sticky=ttk.NSEW)
        ttk.Label(self.model_frame, textvariable=self.model_tc, width=15).grid(row=2, column=2, padx=10, pady=10, sticky=ttk.NSEW)
        ttk.Label(self.model_frame, textvariable=self.model_dt, width=15).grid(row=3, column=2, padx=10, pady=10, sticky=ttk.NSEW)
        ttk.Label(self.model_frame, textvariable=self.model_bias, width=15).grid(row=4, column=2, padx=10, pady=10, sticky=ttk.NSEW)

        ttk.Label(self.tune_frame, textvariable=self.kp_unit).grid(row=0, column=1, columnspan=1, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.tune_frame, textvariable=self.ki_unit).grid(row=0, column=2, columnspan=1, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.tune_frame, textvariable=self.kd_unit).grid(row=0, column=3, columnspan=1, padx=10, pady=10, sticky=ttk.W)

        ttk.Label(self.tune_frame, text="PID Gains:").grid(row=1, column=0, columnspan=1, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.tune_frame, textvariable=self.kp).grid(row=1, column=1, columnspan=1, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.tune_frame, textvariable=self.ki).grid(row=1, column=2, columnspan=1, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.tune_frame, textvariable=self.kd).grid(row=1, column=3, columnspan=1, padx=10, pady=10, sticky=ttk.W)

        ttk.Separator(self.tune_frame, orient=ttk.HORIZONTAL, bootstyle=ttk.DARK).grid(row=2, column=0, columnspan=4, padx=10, pady=10, sticky=ttk.EW)

        ttk.Label(self.tune_frame, text="Tune Method:").grid(row=3, column=0, columnspan=1, padx=10, pady=10, sticky=ttk.W)
        self.tune_method = ttk.StringVar(value="chr")
        self.tune_method_chr = ttk.Radiobutton(self.tune_frame, text="CHR", variable=self.tune_method, value="chr", bootstyle="primary", command=self.update_gains_on_gui)
        self.tune_method_chr.grid(row=3, column=1, padx=10, pady=10, sticky=ttk.NSEW)
        self.tune_method_imc = ttk.Radiobutton(self.tune_frame, text="IMC", variable=self.tune_method, value="imc", command=self.update_gains_on_gui)
        self.tune_method_imc.grid(row=3, column=2, padx=10, pady=10, sticky=ttk.NSEW)
        self.tune_method_aimc = ttk.Radiobutton(self.tune_frame, text="AIMC", variable=self.tune_method, value="aimc", command=self.update_gains_on_gui)
        self.tune_method_aimc.grid(row=3, column=3, padx=10, pady=10, sticky=ttk.NSEW)

        ttk.Label(self.lower_frame, text="Status:").grid(row=0, column=0, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.lower_frame, textvariable=self.gui_status).grid(row=0, column=1, columnspan=1, padx=10, pady=10, sticky=ttk.W)

    def open_file(self):
        self.open_file_button.configure(state=ttk.DISABLED)
        self.generate_model_button.configure(state=ttk.DISABLED)
        self.sim_pid_button.configure(state=ttk.DISABLED)
        thread = threading.Thread(target=self.open_file_thread, name="Open_File_Thread", daemon=True)
        thread.start()

    def open_file_thread(self):
        filename = self.file_name.get()
        if not filename:
            initial_directory = os.getcwd()
        else:
            initial_directory = os.path.dirname(filename)
        filetypes = (("CSV files", "*.csv"), ("All files", "*.*"))
        new_filename = fd.askopenfilename(title="Open a file", initialdir=initial_directory, filetypes=filetypes)
        if new_filename:
            self.file_name.set(new_filename)
            self._reset_values_on_gui()
            self.gui_status.set(f"Loading Data from File")
            self._get_csv_data()

        self.root.after(0, self._change_gui_state, ttk.NORMAL)

    def change_sample_rate(self, event):
        if self.original_sample_rate == self.sample_rate.get():
            return
        else:
            self.original_sample_rate = self.sample_rate.get()
        if not self.file_name.get():
            return

        thread = threading.Thread(target=self._get_csv_data, name="Open_File_Thread", daemon=True)
        thread.start()
        self._reset_values_on_gui()
        self._change_gui_state(ttk.DISABLED)
        self.gui_status.set(f"Updating Sample Rate")

    def _get_csv_data(self):
        try:
            if not self.file_name.get():
                return

            data = np.genfromtxt(self.file_name.get(), delimiter=self.csv_sep.get(), dtype=None, encoding=None, names=True)
            headers = data.dtype.names

            cv_col_name = next((col for col in headers if "CV" in col.upper()), headers[0])
            pv_col_name = next((col for col in headers if "PV" in col.upper()), headers[0])
            auto_col_name = next((col for col in headers if "AUTO" in col.upper()), cv_col_name)
            sp_col_name = next((col for col in headers if "SP" in col.upper()), cv_col_name)

            sample_step = int(1000 / self.sample_rate.get())

            self.pv_array = data[pv_col_name][::sample_step]
            self.cv_array = data[cv_col_name][::sample_step]
            self.auto_array = data[auto_col_name][::sample_step]
            self.sp_array = data[sp_col_name][::sample_step]

        except Exception as e:
            self.gui_status.set(f"Error extracting data from CSV: {str(e)}")

        else:
            self.gui_status.set(f"Data Updated")
            self.root.after(0, self._update_plot)

        finally:
            self.root.after(0, self._change_gui_state, ttk.NORMAL)

    def _load_empty_plot(self):
        try:
            plt.figure(num=1)
            mngr = plt.get_current_fig_manager()
            mngr.window.geometry(f"{int(self.screen_width/2)}x{self.screen_height-self.toolbar}+{int(self.screen_width/2)-self.offset+1}+0")
            plt.xlabel("Time (seconds)")
            plt.ylabel("Value")
            plt.suptitle("Actual Data")
            plt.show(block=False)

        except Exception as e:
            self.gui_status.set(f"Error updating plot: {str(e)}")

    def _update_plot(self):
        try:
            plt.clf()
            plt.gca().legend([])
            plt.plot(self.auto_array, color="purple", linewidth=2, label="Auto")
            plt.plot(self.sp_array, color="goldenrod", linewidth=2, label="SP")
            plt.plot(self.cv_array, color="darkgreen", linewidth=2, label="CV")
            plt.plot(self.pv_array, color="blue", linewidth=2, label="PV")
            if self.model_array is not None:
                plt.plot(self.model_array, color="red", linewidth=2, label="Model")
            plt.xlabel("Time (seconds)")
            plt.ylabel("Value")
            plt.suptitle("Actual Data")
            plt.legend(loc="best")
            plt.show(block=False)

        except Exception as e:
            self.gui_status.set(f"Error updating plot: {str(e)}")

    def solve_for_minimum_error(self):
        self.gui_status.set(f"Attempting to find a Model for the Process")
        self._change_gui_state(ttk.DISABLED)
        thread = threading.Thread(target=self.solve_for_minimum_error_thread, name="Solver_Thread", daemon=True)
        thread.start()

    def solve_for_minimum_error_thread(self):
        try:
            t = np.arange(len(self.pv_array))
            bounds = [(None, None), (0, None), (0, None), (None, None)]
            if self.model_gain.get() == "...":
                initial_model_gain = common.INITIAL_MODEL_GAIN if self.pv_array[0] > self.pv_array[-1] else -common.INITIAL_MODEL_GAIN
                model_values = initial_model_gain, common.INITIAL_MODEL_TC, common.INITIAL_MODEL_DT, common.INITIAL_MODEL_BIAS
            else:
                model_values = float(self.model_gain.get()), float(self.model_tc.get()), float(self.model_dt.get()), float(self.model_bias.get())

            first_model_values = minimize(self.error_function, model_values, args=(t, self.pv_array, self.cv_array), bounds=bounds, method="Nelder-Mead").x
            second_model_values = minimize(self.error_function, first_model_values, args=(t, self.pv_array, self.cv_array), bounds=bounds, method="Nelder-Mead").x
            final_model_values = minimize(self.error_function, second_model_values, args=(t, self.pv_array, self.cv_array), bounds=bounds, method="Nelder-Mead").x
            Gain, Tau, DeadTime, Bias = final_model_values

            # Find Gains
            if self.pid_type == "pid":
                self.gains = common.find_pid_gains(final_model_values)
            elif self.pid_type == "pi":
                self.gains = common.find_pi_gains(final_model_values)

            # Update GUI
            self.model_gain.set(round(Gain, 2))
            self.model_tc.set(round(Tau, 2))
            self.model_dt.set(round(DeadTime, 2))
            self.model_bias.set(round(Bias, 2))
            self.update_gains_on_gui()

            # Generate model with minimized values
            self.model_array = self.fopdt_response(Gain, Tau, DeadTime, Bias, t, self.cv_array)
            self.root.after(0, self._update_plot)

        except Exception as e:
            self.gui_status.set(f"Error finding a Process Model: {str(e)}")

        else:
            self.gui_status.set(f"Model Creation Complete")

        finally:
            self.root.after(0, self._change_gui_state, ttk.NORMAL)

    def sim_pid(self):
        self._change_gui_state(ttk.DISABLED)
        thread = threading.Thread(target=self.sim_pid_thread, name="Sim_Thread", daemon=True)
        thread.start()

    def sim_pid_thread(self):
        try:
            self.gui_status.set(f"Simulating PID loop")
            if self.model_gain.get() == "..." or self.model_tc.get() == "..." or self.model_dt.get() == "..." or self.model_bias.get() == "...":
                self.gui_status.set(f"No Model Values Available")
                return
            if self.kp.get() == "..." or self.ki.get() == "..." or self.kd.get() == "...":
                self.gui_status.set(f"No PID Values Available")
                return

            model_values = float(self.model_gain.get()), float(self.model_tc.get()), float(self.model_dt.get()), float(self.model_bias.get())
            gain_values = float(self.kp.get()), float(self.ki.get()), float(self.kd.get())
            sim_values = common.convert_gains_for_sim(gain_values, pid_form=self.pid_form, pid_time_units=self.pid_time_units)
            ret = common.calculate_pid_response(model_values, sim_values)
            self.root.after(0, self._plot_sim, ret)

        except Exception as e:
            self.gui_status.set(f"Error Simulating PID loop: {str(e)}")

        else:
            self.gui_status.set(f"PID Simulation Complete")

        finally:
            self.root.after(0, self._change_gui_state, ttk.NORMAL)

    def _plot_sim(self, data):
        plt.figure(num=2)
        plt.clf()
        plt.gca().axis("off")
        mngr = plt.get_current_fig_manager()
        mngr.window.geometry(f"{int(self.screen_width/2)}x{self.screen_height-self.toolbar}+{int(self.screen_width/2)-self.offset+1}+0")

        # Setup Plot
        title = f"{self.kp_unit.get()} {self.kp.get()}  {self.ki_unit.get()} {self.ki.get()}  {self.kd_unit.get()} {self.kd.get()}"
        plt.suptitle(f"{self.tune_method.get().upper()} PID Response (ITAE: {round(data[-1])})")
        plt.title(title, fontsize=10)

        # PID Response
        plt.subplot(2, 1, 1)
        plt.plot(data[0], color="goldenrod", linewidth=2, label="SP")
        plt.plot(data[1], color="darkgreen", linewidth=2, label="CV")
        plt.plot(data[2], color="blue", linewidth=2, label="PV")
        plt.ylabel("Value")
        plt.legend(loc="best")

        # Individual components
        plt.subplot(2, 1, 2)
        plt.plot(data[3], color="lime", linewidth=2, label="P Term")
        plt.plot(data[4], color="orange", linewidth=2, label="I Term")
        plt.plot(data[5], color="purple", linewidth=2, label="D Term")
        plt.xlabel("Time (seconds)")
        plt.ylabel("Value")
        plt.legend(loc="best")
        plt.grid(True)
        plt.show(block=False)

    def update_gains_on_gui(self):
        if self.gains and self.model_gain.get() != "...":
            tune_method = self.tune_method.get()
            gains = self.gains[tune_method]
            kp_val = gains["kp"]
            ki_val = gains["ki"]
            kd_val = gains["kd"]

            if self.pid_form == "independent" and self.pid_time_units == "seconds":
                self.kp.set(kp_val)
                self.ki.set(ki_val)
                self.kd.set(kd_val)

            elif self.pid_form == "independent" and self.pid_time_units == "minutes":
                self.kp.set(kp_val)
                self.ki.set(ki_val * 60)
                self.kd.set(round(kd_val / 60, 4))

            elif self.pid_form == "dependent" and self.pid_time_units == "seconds":
                self.kp.set(kp_val)
                self.ki.set(round(kp_val / ki_val, 4))
                self.kd.set(round(kd_val / kp_val, 4))

            elif self.pid_form == "dependent" and self.pid_time_units == "minutes":
                self.kp.set(kp_val)
                self.ki.set(round((kp_val / ki_val) / 60, 4))
                self.kd.set(round((kd_val / kp_val) / 60, 4))

    def _reset_values_on_gui(self):
        self.model_gain.set("...")
        self.model_tc.set("...")
        self.model_dt.set("...")
        self.model_bias.set("...")
        self.kp.set("...")
        self.ki.set("...")
        self.kd.set("...")
        self.model_array = None

        if self.pid_form == "independent" and self.pid_time_units == "seconds":
            self.kp_unit.set("Kp:")
            self.ki_unit.set("Ki (1/sec):")
            self.kd_unit.set("Kd (sec):")

        elif self.pid_form == "independent" and self.pid_time_units == "minutes":
            self.kp_unit.set("Kp:")
            self.ki_unit.set("Ki (1/min):")
            self.kd_unit.set("Kd (min):")

        elif self.pid_form == "dependent" and self.pid_time_units == "seconds":
            self.kp_unit.set("Kc:")
            self.ki_unit.set("Ti (sec/repeat):")
            self.kd_unit.set("Td (sec):")

        elif self.pid_form == "dependent" and self.pid_time_units == "minutes":
            self.kp_unit.set("Kc:")
            self.ki_unit.set("Ti (min/repeat):")
            self.kd_unit.set("Td (min):")

    def _change_gui_state(self, req_state):
        """
        Modify the state (enabled or disabled) of GUI input elements.
        """
        self.sample_rate_entry.configure(state=req_state)
        self.open_file_button.configure(state=req_state)
        if self.file_name.get():
            self.sim_pid_button.configure(state=req_state)
            self.generate_model_button.configure(state=req_state)

    def error_function(self, model_params, t, actual_pv, actual_cv) -> float:
        """
        Calculate the error between the model predictions and actual data.

        Parameters:
            - model_params: Tuple of model parameters (gain, tau, deadtime, bias).
            - t: List of time points.
            - actual_pv: Actual process variable values.
            - actual_cv: Actual control variable values.

        Returns:
            - iae: Integral of absolute error.
        """
        gain, tau, deadtime, bias = model_params
        model_output = self.fopdt_response(gain, tau, deadtime, bias, t, actual_cv)
        iae = np.sum(np.abs(model_output - actual_pv)) * (max(t) - min(t)) / len(t)
        return iae

    def fopdt_response(self, gain, tau, deadtime, bias, t, cv_array) -> np.array:
        """
        Calculate the FOPDT model output for given parameters over time.

        Parameters:
            - gain: Process gain.
            - tau: Time constant.
            - deadtime: Dead time.
            - bias: Bias term.
            - t: Array of time points.
            - cv_array: Control variable values over time.

        Returns:
            - output: Model output values.
        """
        deadtime = max(0.01, deadtime)
        tau = max(0.01, tau)

        offset = np.where(cv_array[:-1] != cv_array[1:])[0]
        t = t - offset[0]

        response = gain * (1 - np.exp(-(t - deadtime) / tau))
        response[t < deadtime] = 0

        calculated_bias = gain * cv_array[0] + bias
        output = response * (cv_array - cv_array[0]) + calculated_bias

        return output


if __name__ == "__main__":
    tuner_app = PIDTuner()
    tuner_app.root.mainloop()
