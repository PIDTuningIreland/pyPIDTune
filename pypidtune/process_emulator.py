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

import time
import random
import threading
from datetime import datetime
import numpy as np
import ttkbootstrap as ttk
import matplotlib.pyplot as plt
from matplotlib import animation
from pylogix import PLC
from asyncua.sync import Client, ua
import common

SAMPLE_RATE = 0.1


class ProcessEmulator:
    """
    ProcessEmulator is responsible for setting up PLC communication and initializing the GUI
    for emulating a FOPDT process.
    """

    def __init__(self, **kwargs):
        """
        Parameters:
        -----------
        `**kwargs`: dict
            A dictionary of optional parameters for configuration.

        Keyword Arguments:
        ------------------
        - `comm_type` : str, optional

            The type of communication to use, either 'logix' or 'opc'. Defaults to 'logix'.

        - `init_logix_ip_address` : str, optional

            The initial IP address for Logix communication. Defaults to a predefined constant.

        - `init_logix_slot` : int, optional

            The initial slot for Logix communication. Defaults to a predefined constant.

        - `init_opc_address` : str, optional

            The initial address for OPC communication. Defaults to a predefined constant.

        - `init_sp_tag` : str, optional

            The initial setpoint tag for the PID controller. Defaults to a predefined constant.

        - `init_pv_tag` : str, optional

            The initial process variable tag for the PID controller. Defaults to a predefined constant.

        - `init_cv_tag` : str, optional

            The initial control variable tag for the PID controller. Defaults to a predefined constant.

        - `init_model_gain` : float, optional

            Initial gain of the process model. Default is 2.5.

        - `init_model_tc` : float, optional

            Initial time constant of the process model. Default is 75.5 seconds.

        - `init_model_dt` : float, optional

            Initial dead time of the process model. Default is 10.5 seconds.

        - `init_model_bias` : float, optional

            Initial bias of the process model. Default is 13.5.
        """
        # Initialize instance attributes based on provided or default values
        self.comm_type = kwargs.get("comm_type", "logix")
        self.init_logix_ip_address = kwargs.get("init_logix_ip_address", common.INITIAL_LOGIX_IP_ADDRESS)
        self.init_logix_slot = kwargs.get("init_logix_slot", common.INITIAL_LOGIX_SLOT)
        self.init_opc_address = kwargs.get("init_opc_address", common.INITIAL_OPC_ADDRESS)
        self.init_sp_tag = kwargs.get("init_sp_tag", common.INITIAL_SP_TAG)
        self.init_pv_tag = kwargs.get("init_pv_tag", common.INITIAL_PV_TAG)
        self.init_cv_tag = kwargs.get("init_cv_tag", common.INITIAL_CV_TAG)
        try:
            self.init_model_gain = float(kwargs.get("init_model_gain", common.INITIAL_MODEL_GAIN))
            self.init_model_tc = float(kwargs.get("init_model_tc", common.INITIAL_MODEL_TC))
            self.init_model_dt = float(kwargs.get("init_model_dt", common.INITIAL_MODEL_DT))
            self.init_model_bias = float(kwargs.get("init_model_bias", common.INITIAL_MODEL_BIAS))
        except ValueError:
            raise ValueError("init_model_gain, init_model_tc, init_model_dt and init_model_bias must all be floats")

        self._build_gui()
        self._initial_setup()
        self._empty_plot_setup()

    def _build_gui(self):
        """
        Set up the graphical user interface (GUI) elements using ttkbootstrap.
        """
        plt.style.use("bmh")
        self.root = ttk.Window()
        self.root.title("Process Emulator  - PID Tuning Ireland©")
        # Adjust theme to suit
        style = ttk.Style(theme="yeti")
        style.theme.colors.bg = "#c0c0c0"

        self.screen_width = self.root.winfo_screenwidth()
        self.screen_height = self.root.winfo_screenheight()
        self.offset = 7
        self.toolbar = 73

        # Configure GUI window size and appearance
        self.root.resizable(True, True)
        self.root.geometry(f"{int(self.screen_width/2)}x{self.screen_height-self.toolbar}+{-self.offset}+0")

        self.main_frame = ttk.Frame(self.root)
        self.main_frame.pack(expand=True, fill=ttk.BOTH)

        # Define various variables to be used in the GUI
        self.sp_value = ttk.DoubleVar()
        self.pv_value = ttk.DoubleVar()
        self.cv_value = ttk.DoubleVar()

        self.read_count = ttk.IntVar()
        self.error_count = ttk.IntVar()
        self.gui_status = ttk.StringVar()

        self.sp_plc_tag = ttk.StringVar(value=self.init_sp_tag)
        self.pv_plc_tag = ttk.StringVar(value=self.init_pv_tag)
        self.cv_plc_tag = ttk.StringVar(value=self.init_cv_tag)

        self.model_gain = ttk.DoubleVar(value=self.init_model_gain)
        self.model_tc = ttk.DoubleVar(value=self.init_model_tc)
        self.model_dt = ttk.DoubleVar(value=self.init_model_dt)
        self.model_bias = ttk.DoubleVar(value=self.init_model_bias)

        if self.comm_type == "logix":
            self.plc_address = ttk.StringVar(value=self.init_logix_ip_address)
        elif self.comm_type == "opc":
            self.plc_address = ttk.StringVar(value=self.init_opc_address)

        self.slot = ttk.IntVar(value=self.init_logix_slot)

        self.tags_frame = ttk.LabelFrame(self.main_frame, padding=5, text="Control")
        self.tags_frame.pack(expand=True, fill=ttk.BOTH, padx=5, pady=5)
        self.settings_frame = ttk.LabelFrame(self.main_frame, padding=5, text="Settings")
        self.settings_frame.pack(expand=True, fill=ttk.BOTH, padx=5, pady=5)

        # Orgainse
        self.top_tags_frame = ttk.Frame(self.tags_frame)
        self.top_tags_frame.pack(expand=True, fill=ttk.BOTH)
        self.bottom_tags_frame = ttk.Frame(self.tags_frame)
        self.bottom_tags_frame.pack(expand=True, fill=ttk.BOTH)

        # Column 0: Labels
        ttk.Label(self.top_tags_frame, text="SP:").grid(row=1, column=0, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.top_tags_frame, text="PV:").grid(row=2, column=0, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.top_tags_frame, text="CV:").grid(row=3, column=0, padx=10, pady=10, sticky=ttk.W)

        # Column 1: PLC Tags
        ttk.Label(self.top_tags_frame, text="PLC Tag").grid(row=0, column=1, padx=10, pady=10, sticky=ttk.W)
        self.entry_sp_tag = ttk.Entry(self.top_tags_frame, textvariable=self.sp_plc_tag, width=50)
        self.entry_sp_tag.grid(row=1, column=1, padx=10, pady=10, sticky=ttk.NSEW)
        self.entry_pv_tag = ttk.Entry(self.top_tags_frame, textvariable=self.pv_plc_tag, width=50)
        self.entry_pv_tag.grid(row=2, column=1, padx=10, pady=10, sticky=ttk.NSEW)
        self.entry_cv_tag = ttk.Entry(self.top_tags_frame, textvariable=self.cv_plc_tag, width=50)
        self.entry_cv_tag.grid(row=3, column=1, padx=10, pady=10, sticky=ttk.NSEW)

        # Column 2: Actual Values
        ttk.Label(self.top_tags_frame, text="    Value    ").grid(row=0, column=2, padx=10, pady=10)
        ttk.Label(self.top_tags_frame, textvariable=self.sp_value).grid(row=1, column=2, padx=10, pady=10)
        ttk.Label(self.top_tags_frame, textvariable=self.pv_value).grid(row=2, column=2, padx=10, pady=10)
        ttk.Label(self.top_tags_frame, textvariable=self.cv_value).grid(row=3, column=2, padx=10, pady=10)

        # Buttons
        self.button_start = ttk.Button(self.bottom_tags_frame, width=25, text="Start Simulator", command=lambda: [self.start()])
        self.button_start.grid(row=0, column=0, columnspan=1, padx=10, pady=10, sticky=ttk.NSEW)

        self.button_stop = ttk.Button(self.bottom_tags_frame, width=25, text="Stop Simulator", command=lambda: [self.stop()])
        self.button_stop.grid(row=0, column=1, columnspan=1, padx=10, pady=10, sticky=ttk.NSEW)
        self.button_stop.configure(state=ttk.DISABLED)

        self.button_show_trend = ttk.Button(self.bottom_tags_frame, text="Show Trend", command=lambda: [self.show_live_trend()])
        self.button_show_trend.grid(row=1, column=0, columnspan=2, padx=10, pady=10, sticky=ttk.NSEW)
        self.button_show_trend.configure(state=ttk.DISABLED)

        # Settings Frame
        ttk.Label(self.settings_frame, text="PLC Address:").grid(row=0, column=0, padx=10, pady=10, sticky=ttk.W)
        self.entry_plc_address = ttk.Entry(self.settings_frame, width=25, textvariable=self.plc_address)
        self.entry_plc_address.grid(row=0, column=1, columnspan=2, padx=10, pady=10, sticky=ttk.NSEW)

        ttk.Label(self.settings_frame, text="PLC Slot:").grid(row=1, column=0, padx=10, pady=10, sticky=ttk.W)
        self.entry_slot = ttk.Entry(self.settings_frame, textvariable=self.slot, width=6)
        self.entry_slot.grid(row=1, column=1, padx=10, pady=10, sticky=ttk.W)

        ttk.Label(self.settings_frame, text="Interval (mS):").grid(row=2, column=0, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.settings_frame, text="100").grid(row=2, column=1, padx=10, pady=10, sticky=ttk.W)

        ttk.Label(self.settings_frame, text="Read Count:").grid(row=3, column=0, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.settings_frame, textvariable=self.read_count).grid(row=3, column=1, padx=10, pady=10, sticky=ttk.W)

        ttk.Label(self.settings_frame, text="Error Count:").grid(row=4, column=0, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.settings_frame, textvariable=self.error_count).grid(row=4, column=1, padx=10, pady=10, sticky=ttk.W)

        ttk.Label(self.settings_frame, text="Status:").grid(row=5, column=0, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.settings_frame, textvariable=self.gui_status).grid(row=5, column=1, columnspan=5, padx=10, pady=10, sticky=ttk.W)

        ttk.Label(self.settings_frame, text="First Order Plus Dead Time Process Model").grid(row=0, column=3, columnspan=2, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.settings_frame, text="Model Gain: ").grid(row=1, column=3, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.settings_frame, text="Time Constant (seconds):").grid(row=2, column=3, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.settings_frame, text="Dead Time (seconds):").grid(row=3, column=3, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.settings_frame, text="Bias:").grid(row=4, column=3, padx=10, pady=10, sticky=ttk.W)

        self.model_gain_spinbox = ttk.Spinbox(self.settings_frame, from_=-1000.00, to=1000.00, increment=0.1, textvariable=self.model_gain, width=6)
        self.model_gain_spinbox.grid(row=1, column=4, padx=10, pady=10)
        self.model_tc_spinbox = ttk.Spinbox(self.settings_frame, from_=1.00, to=1000.00, increment=0.1, textvariable=self.model_tc, width=6)
        self.model_tc_spinbox.grid(row=2, column=4, padx=10, pady=10)
        self.model_dt_spinbox = ttk.Spinbox(self.settings_frame, from_=1.00, to=1000.00, increment=0.1, textvariable=self.model_dt, width=6)
        self.model_dt_spinbox.grid(row=3, column=4, padx=10, pady=10)
        self.model_bias_spinbox = ttk.Spinbox(self.settings_frame, from_=-1000.00, to=1000.00, increment=0.1, textvariable=self.model_bias, width=6)
        self.model_bias_spinbox.grid(row=4, column=4, padx=10, pady=10)

    def _initial_setup(self):
        """
        Initialize necessary resources like threads, PLC/OPC communication objects,
        CSV file handling, and client connections.
        """
        self.thread_stop_event = threading.Event()
        self.thread_lock = threading.Lock()
        self.PV = np.zeros(0)
        self.CV = np.zeros(0)
        self.SP = np.zeros(0)
        self.scan_count = 0
        self.comm = None
        self.anim = None
        self.looped_thread_for_data_retrieval = None

        if self.comm_type == "logix":
            self.comm = PLC(ip_address=self.plc_address.get(), slot=self.slot.get())
        elif self.comm_type == "opc":
            self.comm = Client(self.plc_address.get())
        self.process = common.FOPDTModel()

    def _empty_plot_setup(self):
        """
        Configure an empty plot window using matplotlib.
        """
        self.fig = plt.figure()
        self.ax = plt.axes()
        (self.plot_SP,) = self.ax.plot([], [], color="goldenrod", linewidth=2, label="SP")
        (self.plot_CV,) = self.ax.plot([], [], color="darkgreen", linewidth=2, label="CV")
        (self.plot_PV,) = self.ax.plot([], [], color="blue", linewidth=2, label="PV")
        plt.ylabel("EU")
        plt.xlabel("Time (min)")
        plt.suptitle("Live Data")
        plt.legend(loc="upper right")
        mngr = plt.get_current_fig_manager()
        mngr.window.geometry(f"{int(self.screen_width/2)}x{self.screen_height-self.toolbar}+{int(self.screen_width/2)-self.offset+1}+0")
        plt.gcf().canvas.mpl_connect("close_event", self._on_plot_close)
        plt.show(block=False)

    def start(self):
        self.thread_stop_event.clear()
        self.PV = np.zeros(0)
        self.CV = np.zeros(0)
        self.SP = np.zeros(0)
        self.scan_count = 0
        self.error_count.set(0)
        self.read_count.set(0)
        self.gui_status.set("")
        self.button_stop.configure(state=ttk.NORMAL)
        self.button_start.configure(state=ttk.DISABLED)
        self.button_show_trend.configure(state=ttk.DISABLED)
        self._change_gui_state(ttk.DISABLED)
        thread = threading.Thread(target=self.pre_flight_checks, name="Pre_Flight_Checks_Thread", daemon=True)
        thread.start()

    def _process_pre_flight_result(self, pre_flight_checks_completed):
        if pre_flight_checks_completed:
            self.looped_thread_for_data_retrieval = common.PeriodicInterval(self.data_retrieval, SAMPLE_RATE)
            if not 1 in plt.get_fignums():
                self._empty_plot_setup()
            self.live_trend()

        elif not pre_flight_checks_completed or self.thread_stop_event.is_set():
            self.button_stop.configure(state=ttk.DISABLED)
            self.button_start.configure(state=ttk.NORMAL)
            self._change_gui_state(ttk.NORMAL)

    def pre_flight_checks(self):
        """
        Perform pre-flight checks before starting data recording:
        - Test PLC/OPC communication
        - Verify tag names and configuration
        """
        try:
            pre_flight_checks_completed = False
            self.process.Gain = float(self.model_gain.get())
            self.process.Time_Constant = float(self.model_tc.get()) / SAMPLE_RATE
            self.process.Dead_Time = float(self.model_dt.get()) / SAMPLE_RATE
            self.process.Bias = float(self.model_bias.get())

            # Check PLC communication settings and prepare for recording
            self.tag_list_for_reads = [self.sp_plc_tag.get(), self.cv_plc_tag.get()]
            self.tag_list_for_writes = [self.pv_plc_tag.get()]
            self.full_tag_list = self.tag_list_for_reads + self.tag_list_for_writes

            if self.comm_type == "logix":
                self.comm = PLC(ip_address=self.plc_address.get(), slot=self.slot.get())
                self.comm.IPAddress = self.plc_address.get()
                self.comm.ProcessorSlot = self.slot.get()
                self.comm.SocketTimeout = 10.0
                test_comms = self.comm.GetPLCTime()
                if test_comms.Status != "Success":
                    raise Exception("Cannot Read Logix Clock - Check Configuration")

                # Test Read
                ret_response = self.comm.Read(self.full_tag_list)
                ret = [x.Value for x in ret_response]
                self.comm.SocketTimeout = sorted([0.1, SAMPLE_RATE * 1.1, 5.0])[1]

            elif self.comm_type == "opc":
                try:
                    self.comm = Client(url=self.plc_address.get())
                    self.comm.connect()
                except:
                    raise Exception("Failed to Create OPC Connection - Check Configuration")
                else:
                    # Test Read
                    self.full_tag_list = [self.comm.get_node(x) for x in self.full_tag_list]
                    ret = self.comm.read_values(self.full_tag_list)
                    self.tag_list_for_reads = [self.comm.get_node(x) for x in self.tag_list_for_reads]
                    self.tag_list_for_writes = [self.comm.get_node(x) for x in self.tag_list_for_writes]

            if any(value is None for value in ret):
                raise Exception("Tag Name Incorrect - Check Configuration")

            if self.thread_stop_event.is_set():
                return

        except Exception as e:
            current_date_time = datetime.now().strftime("%d-%m-%Y %H:%M:%S")
            self.gui_status.set(f"{current_date_time} - Pre Flight Error: {str(e)}")

        else:
            pre_flight_checks_completed = True

        finally:
            self.root.after(0, self._process_pre_flight_result, pre_flight_checks_completed)

    def data_retrieval(self):
        """
        Retrieve data from PLC/OPC, update GUI values, write data to CSV,
        and handle exceptions during data retrieval.
        """
        try:
            # Check if stop is requested
            if self.thread_stop_event.is_set():
                return

            # Read data from PLC, update GUI, and write to CSV
            if self.comm_type == "logix":
                ret_response = self.comm.Read(self.tag_list_for_reads)
                ret = [x.Value for x in ret_response]

            elif self.comm_type == "opc":
                ret = self.comm.read_values(self.tag_list_for_reads)

            if all(x is not None for x in ret):
                self.sp_value.set(round(ret[0], 2))
                self.cv_value.set(round(ret[1], 2))
                self.SP = np.append(self.SP, ret[0])
                self.CV = np.append(self.CV, ret[1])
                self.read_count.set(self.read_count.get() + 1)

            elif not self.thread_stop_event.is_set():
                self.error_count.set(self.error_count.get() + 1)
                current_date_time = datetime.now().strftime("%d-%m-%Y %H:%M:%S")
                self.gui_status.set(f"{current_date_time} - Error Getting Data")

            # Send CV to Process
            self.process.stored_cv = self.CV
            ts = [self.scan_count, self.scan_count + 1]

            # Get new PV value
            if self.PV.size > 1:
                pv = self.process.update(self.PV[-1], ts)
            else:
                pv = self.process.update(float(self.model_bias.get()), ts)

            # Add Noise
            noise = random.uniform(-0.01, 0.01)
            pv_and_noise = pv + noise

            # Store PV
            self.PV = np.append(self.PV, pv_and_noise)

            # Check if stop is requested
            if self.thread_stop_event.is_set():
                return

            # Write PV to PLC
            if self.comm_type == "logix":
                write = self.comm.Write(self.pv_plc_tag.get(), pv_and_noise)
                if write.Status != "Success":
                    raise Exception("Write Failed")

            elif self.comm_type == "opc":
                self.tag_list_for_writes[0].write_attribute(ua.AttributeIds.Value, ua.DataValue(float(pv_and_noise)))

            self.pv_value.set(round(pv_and_noise, 2))

        except Exception as e:
            current_date_time = datetime.now().strftime("%d-%m-%Y %H:%M:%S")
            self.gui_status.set(f"{current_date_time} - Error Retrieving Data: {str(e)}")

        finally:
            self.scan_count += 1

    def _change_gui_state(self, req_state):
        """
        Modify the state (enabled or disabled) of GUI input elements.
        """
        self.entry_sp_tag.configure(state=req_state)
        self.entry_pv_tag.configure(state=req_state)
        self.entry_cv_tag.configure(state=req_state)
        self.entry_plc_address.configure(state=req_state)
        self.entry_slot.configure(state=req_state)
        self.model_gain_spinbox.configure(state=req_state)
        self.model_tc_spinbox.configure(state=req_state)
        self.model_dt_spinbox.configure(state=req_state)
        self.model_bias_spinbox.configure(state=req_state)

    def live_trend(self):
        # Setup Plot
        def init():
            if not 1 in plt.get_fignums():
                self._empty_plot_setup()

        # Loop here
        def animate(i):
            with self.thread_lock:
                x = np.arange(len(self.SP)) / 600
                if len(x) == len(self.SP) == len(self.PV) == len(self.CV):
                    self.plot_SP.set_data(x, self.SP)
                    self.plot_CV.set_data(x, self.CV)
                    self.plot_PV.set_data(x, self.PV)
                    self.ax.relim()
                    self.ax.autoscale_view()

        # Live Data
        self.anim = animation.FuncAnimation(self.fig, animate, init_func=init, frames=60, interval=1000)
        plt.show()

    def _on_plot_close(self, event):
        """
        Stop Animation when the Live Plot is closed.
        Enable Button to allow plot to be re-opened.
        """
        if self.anim:
            self.anim.event_source.stop()

        if self.looped_thread_for_data_retrieval:
            self.button_show_trend.configure(state=ttk.NORMAL)

    def show_live_trend(self):
        """
        Control the display of the live trend plot window.
        """
        self.button_show_trend.configure(state=ttk.DISABLED)
        plot_open = 1 in plt.get_fignums()
        if not plot_open:
            self._empty_plot_setup()
            self.live_trend()

    def stop(self):
        """
        Stop the data recording process, reset GUI elements, close files,
        disconnect from PLC/OPC server, and handle errors during shutdown.
        """
        try:
            # Stop data recording, reset GUI elements, and close files
            self.thread_stop_event.set()
            if self.looped_thread_for_data_retrieval:
                self.looped_thread_for_data_retrieval.stop()

            self._change_gui_state(req_state=ttk.NORMAL)
            self.button_show_trend.configure(state=ttk.DISABLED)
            self.button_start.configure(state=ttk.NORMAL)

            self.anim.event_source.stop()
            if self.comm:
                if self.comm_type == "logix":
                    self.comm.Close()
                elif self.comm_type == "opc":
                    self.comm.disconnect()

        except Exception as e:
            self.gui_status.set("Stop Error: " + str(e))

        else:
            self.button_stop.configure(state=ttk.DISABLED)


if __name__ == "__main__":
    emulator_app = ProcessEmulator()
    emulator_app.root.mainloop()
