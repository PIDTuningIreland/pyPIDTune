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
import csv
import time
import threading
from datetime import datetime
import numpy as np
import ttkbootstrap as ttk
import matplotlib.pyplot as plt
from matplotlib import animation
from pylogix import PLC
from asyncua.sync import Client, ua
import common


class PIDLogger:
    """
    PIDLogger is responsible for setting up PLC communication and initializing the GUI
    for monitoring and logging PID controller data.
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
        """
        # Initialize instance attributes based on provided or default values
        self.comm_type = kwargs.get("comm_type", "logix")
        self.init_logix_ip_address = kwargs.get("init_logix_ip_address", common.INITIAL_LOGIX_IP_ADDRESS)
        self.init_logix_slot = kwargs.get("init_logix_slot", common.INITIAL_LOGIX_SLOT)
        self.init_opc_address = kwargs.get("init_opc_address", common.INITIAL_OPC_ADDRESS)
        self.init_sp_tag = kwargs.get("init_sp_tag", common.INITIAL_SP_TAG)
        self.init_pv_tag = kwargs.get("init_pv_tag", common.INITIAL_PV_TAG)
        self.init_cv_tag = kwargs.get("init_cv_tag", common.INITIAL_CV_TAG)

        # Initialize GUI setup, initial setup, and empty plot setup
        self._build_gui()
        self._initial_setup()
        self._empty_plot_setup()

    def _build_gui(self):
        """
        Set up the graphical user interface (GUI) elements using ttkbootstrap.
        """
        plt.style.use("bmh")
        self.root = ttk.Window()
        self.root.title("PID Data Logger - PID Tuning Ireland©")
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

        self.sp_write_value = ttk.StringVar()
        self.cv_write_value = ttk.StringVar()

        self.slot = ttk.IntVar(value=self.init_logix_slot)

        if self.comm_type == "logix":
            self.plc_address = ttk.StringVar(value=self.init_logix_ip_address)
        elif self.comm_type == "opc":
            self.plc_address = ttk.StringVar(value=self.init_opc_address)

        self.delta_t = ttk.DoubleVar(value=100)
        self.file_name = ttk.StringVar(value=r"D:\Trend.csv")

        self.tags_frame = ttk.LabelFrame(self.main_frame, padding=5, text="Control")
        self.tags_frame.pack(expand=True, fill=ttk.BOTH, padx=5, pady=5)
        self.settings_frame = ttk.LabelFrame(self.main_frame, padding=5, text="Settings")
        self.settings_frame.pack(expand=True, fill=ttk.BOTH, padx=5, pady=5)

        # Column 0: Labels
        ttk.Label(self.tags_frame, text="Tag").grid(row=0, column=0, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.tags_frame, text="SP:").grid(row=1, column=0, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.tags_frame, text="PV:").grid(row=2, column=0, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.tags_frame, text="CV:").grid(row=3, column=0, padx=10, pady=10, sticky=ttk.W)

        # Column 1: PLC Tags
        ttk.Label(self.tags_frame, text="PLC Tag").grid(row=0, column=1, padx=10, pady=10, sticky=ttk.W)
        self.entry_sp_tag = ttk.Entry(self.tags_frame, textvariable=self.sp_plc_tag)
        self.entry_sp_tag.grid(row=1, column=1, padx=10, pady=10, sticky=ttk.NSEW)
        self.entry_pv_tag = ttk.Entry(self.tags_frame, textvariable=self.pv_plc_tag)
        self.entry_pv_tag.grid(row=2, column=1, padx=10, pady=10, sticky=ttk.NSEW)
        self.entry_cv_tag = ttk.Entry(self.tags_frame, textvariable=self.cv_plc_tag)
        self.entry_cv_tag.grid(row=3, column=1, padx=10, pady=10, sticky=ttk.NSEW)

        # Column 2: Actual Values
        ttk.Label(self.tags_frame, text="      Value      ").grid(row=0, column=2, padx=10, pady=10)
        ttk.Label(self.tags_frame, textvariable=self.sp_value).grid(row=1, column=2, padx=10, pady=10)
        ttk.Label(self.tags_frame, textvariable=self.pv_value).grid(row=2, column=2, padx=10, pady=10)
        ttk.Label(self.tags_frame, textvariable=self.cv_value).grid(row=3, column=2, padx=10, pady=10)

        # Column 4: Send - Write Values
        ttk.Label(self.tags_frame, text="Write to PLC").grid(row=0, column=4, padx=10, pady=10, sticky=ttk.NSEW)
        self.entry_sp_write = ttk.Entry(self.tags_frame, textvariable=self.sp_write_value)
        self.entry_sp_write.grid(row=1, column=4, padx=10, pady=10, sticky=ttk.NSEW)
        self.entry_cv_write = ttk.Entry(self.tags_frame, textvariable=self.cv_write_value)
        self.entry_cv_write.grid(row=3, column=4, padx=10, pady=10, sticky=ttk.NSEW)

        # Buttons
        self.button_start = ttk.Button(self.tags_frame, text="Record Data", command=lambda: [self.start()])
        self.button_start.grid(row=4, column=1, columnspan=2, padx=10, pady=10, sticky=ttk.NSEW)

        self.button_livetrend = ttk.Button(self.tags_frame, text="Live Plot", command=lambda: [self.show_live_trend()])
        self.button_livetrend.grid(row=4, column=3, columnspan=1, padx=10, pady=10, sticky=ttk.NSEW)
        self.button_livetrend.configure(state=ttk.DISABLED)

        self.button_write = ttk.Button(self.tags_frame, text="Write", command=lambda: [self.write_to_PLC()])
        self.button_write.grid(row=4, column=4, columnspan=1, padx=10, pady=10, sticky=ttk.NSEW)

        self.button_stop = ttk.Button(self.tags_frame, text="Stop Recording", command=lambda: [self.stop()])
        self.button_stop.grid(row=5, column=1, columnspan=3, padx=10, pady=10, sticky=ttk.NSEW)
        self.button_stop.configure(state=ttk.DISABLED)

        self.button_show_file_data = ttk.Button(self.tags_frame, text="Plot Data From CSV", command=lambda: [self.open_trend_from_file()])
        self.button_show_file_data.grid(row=5, column=4, columnspan=1, padx=10, pady=10, sticky=ttk.NSEW)

        # Settings
        ttk.Label(self.settings_frame, text="PLC Address:").grid(row=0, column=0, padx=10, pady=10, sticky=ttk.W)
        self.entry_plc_address = ttk.Entry(self.settings_frame, textvariable=self.plc_address)
        self.entry_plc_address.grid(row=0, column=1, columnspan=2, padx=10, pady=10, sticky=ttk.NSEW)

        ttk.Label(self.settings_frame, text="PLC Slot:").grid(row=1, column=0, padx=10, pady=10, sticky=ttk.W)
        self.entry_slot = ttk.Entry(self.settings_frame, textvariable=self.slot, width=6)
        self.entry_slot.grid(row=1, column=1, padx=10, pady=10, sticky=ttk.W)

        ttk.Label(self.settings_frame, text="Interval (mS):").grid(row=2, column=0, padx=10, pady=10, sticky=ttk.W)
        self.entry_delta_t = ttk.Entry(self.settings_frame, textvariable=self.delta_t, width=6)
        self.entry_delta_t.grid(row=2, column=1, columnspan=1, padx=10, pady=10, sticky=ttk.W)

        ttk.Label(self.settings_frame, text="File Name:").grid(row=3, column=0, padx=10, pady=10, sticky=ttk.W)
        self.entry_file_name = ttk.Entry(self.settings_frame, textvariable=self.file_name)
        self.entry_file_name.grid(row=3, column=1, columnspan=3, padx=10, pady=10, sticky=ttk.NSEW)

        ttk.Label(self.settings_frame, text="Read Count:").grid(row=4, column=0, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.settings_frame, textvariable=self.read_count).grid(row=4, column=1, padx=10, pady=10, sticky=ttk.W)

        ttk.Label(self.settings_frame, text="Error Count:").grid(row=5, column=0, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.settings_frame, textvariable=self.error_count).grid(row=5, column=1, padx=10, pady=10, sticky=ttk.W)

        ttk.Label(self.settings_frame, text="Status:").grid(row=6, column=0, padx=10, pady=10, sticky=ttk.W)
        ttk.Label(self.settings_frame, textvariable=self.gui_status).grid(row=6, column=1, columnspan=5, padx=10, pady=10, sticky=ttk.W)

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
        self.looped_thread_for_data_retrieval = None
        self.csv_file = None
        self.csv_file_writer = None
        self.comm = None
        self.comm_write = None
        self.anim = None
        self.scale = None
        if self.comm_type == "logix":
            self.comm = PLC(ip_address=self.plc_address.get(), slot=self.slot.get())
        elif self.comm_type == "opc":
            self.comm = Client(self.plc_address.get())

    def _empty_plot_setup(self):
        """
        Configure an empty plot window using matplotlib.
        """
        self.fig = plt.figure(num=1)
        self.ax = plt.axes()
        (self.plot_SP,) = self.ax.plot([], [], color="goldenrod", linewidth=2, label="SP")
        (self.plot_CV,) = self.ax.plot([], [], color="darkgreen", linewidth=2, label="CV")
        (self.plot_PV,) = self.ax.plot([], [], color="blue", linewidth=2, label="PV")
        plt.ylabel("Value")
        plt.xlabel("Time (Minutes)")
        plt.suptitle("Live Data")
        plt.legend(loc="upper right")
        self.scale = int(60000 / int(self.delta_t.get()))
        mngr = plt.get_current_fig_manager()
        mngr.window.geometry(f"{int(self.screen_width/2)}x{self.screen_height-self.toolbar}+{int(self.screen_width/2)-self.offset+1}+0")
        plt.gcf().canvas.mpl_connect("close_event", self._on_plot_close)
        plt.show(block=False)

    def start(self):
        """
        Start data recording process:
        - Clear previous data
        - Update GUI state
        - Start a new thread for Pre-Flight checks
        """
        self.thread_stop_event.clear()
        self.PV = np.zeros(0)
        self.CV = np.zeros(0)
        self.SP = np.zeros(0)
        self.error_count.set(0)
        self.read_count.set(0)
        self.gui_status.set("")
        self.button_stop.configure(state=ttk.NORMAL)
        self.button_start.configure(state=ttk.DISABLED)
        self._change_gui_state(ttk.DISABLED)
        thread = threading.Thread(target=self.pre_flight_checks, name="Pre_Flight_Checks_Thread", daemon=True)
        thread.start()

    def _process_pre_flight_result(self, pre_flight_checks_completed):
        if pre_flight_checks_completed:
            self.looped_thread_for_data_retrieval = common.PeriodicInterval(self.data_retrieval, int(self.delta_t.get()) / 1000)
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
        - Open CSV file for data logging
        - Verify tag names and configuration
        """

        # Check PLC communication settings and prepare for recording
        try:
            pre_flight_checks_completed = False
            self.tag_list = [self.sp_plc_tag.get(), self.pv_plc_tag.get(), self.cv_plc_tag.get()]
            if self.comm_type == "logix":
                self.comm = PLC(ip_address=self.plc_address.get(), slot=self.slot.get())
                self.comm.IPAddress = self.plc_address.get()
                self.comm.ProcessorSlot = self.slot.get()
                self.comm.SocketTimeout = 10.0
                test_comms = self.comm.GetPLCTime()
                if test_comms.Status != "Success":
                    raise Exception("Cannot Read Logix Clock - Check Configuration")
            elif self.comm_type == "opc":
                try:
                    self.comm = Client(url=self.plc_address.get())
                    self.comm.connect()
                except:
                    raise Exception("Failed to Create OPC Connection - Check Configuration")

            if self.thread_stop_event.is_set():
                return

            # Open CSV File
            self.csv_file = open(self.file_name.get(), "a")
            self.csv_file_writer = csv.writer(self.csv_file, delimiter=";", lineterminator="\n", quotechar="/", quoting=csv.QUOTE_MINIMAL)
            if os.stat(self.file_name.get()).st_size == 0:
                self.csv_file_writer.writerow(("SP", "PV", "CV", "TimeStamp"))

            if self.comm_type == "logix":
                ret_response = self.comm.Read(self.tag_list)
                ret = [x.Value for x in ret_response]
                self.comm.SocketTimeout = sorted([0.1, self.delta_t.get() * 1.1 / 1000, 5.0])[1]

            elif self.comm_type == "opc":
                self.tag_list = [self.comm.get_node(x) for x in self.tag_list]
                ret = self.comm.read_values(self.tag_list)

            if any(value is None for value in ret):
                raise Exception("Tag Name Incorrect - Check Configuration")

            if self.thread_stop_event.is_set():
                return

        except Exception as e:
            current_date_time = datetime.now().strftime("%d-%m-%Y %H:%M:%S")
            self.gui_status.set(f"{current_date_time} - Pre-Flight Error: {str(e)}")

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
            # Read data from PLC, update GUI, and write to CSV
            if self.comm_type == "logix":
                ret_response = self.comm.Read(self.tag_list)
                ret = [x.Value for x in ret_response]
            elif self.comm_type == "opc":
                ret = self.comm.read_values(self.tag_list)

            gui_tags = [self.sp_value, self.pv_value, self.cv_value]

            if self.thread_stop_event.is_set():
                return

            if all(x is not None for x in ret):
                current_date_time_ms = np.datetime64("now", "ms")
                for tag, value in zip(gui_tags, ret):
                    tag.set(round(value, 3))
                self.csv_file_writer.writerow(ret + [current_date_time_ms])
                self.SP, self.PV, self.CV = np.append(self.SP, ret[0]), np.append(self.PV, ret[1]), np.append(self.CV, ret[2])
                self.read_count.set(self.read_count.get() + 1)
            else:
                self.error_count.set(self.error_count.get() + 1)
                current_date_time = datetime.now().strftime("%d-%m-%Y %H:%M:%S")
                self.gui_status.set(f"{current_date_time} - Error Getting Data")

        except Exception as e:
            current_date_time = datetime.now().strftime("%d-%m-%Y %H:%M:%S")
            self.gui_status.set(f"{current_date_time} - Error: {str(e)}")

    def write_to_PLC(self):
        """
        Write setpoint (SP) and control variable (CV) values to the PLC/OPC server.
        """
        if self.sp_write_value.get() == "" and self.cv_write_value.get() == "":
            self.gui_status.set("No Values to Write")
            return

        try:
            # Write values to the PLC
            if not self.comm_write:
                if self.comm_type == "logix":
                    self.comm_write = PLC()
                    self.comm_write.IPAddress = self.plc_address.get()
                    self.comm_write.ProcessorSlot = self.slot.get()
                    self.comm_write.SocketTimeout = 1
                elif self.comm_type == "opc":
                    self.comm_write = Client(url=self.plc_address.get())
                    self.comm_write.connect()

            if self.sp_write_value.get() != "":
                sp = float(self.sp_write_value.get())
                if self.comm_type == "logix":
                    ret = self.comm_write.Write(self.sp_plc_tag.get(), sp)
                    if ret.Status != "Success":
                        raise Exception(ret.TagName + " " + ret.Status)
                elif self.comm_type == "opc":
                    tag = self.comm_write.get_node(self.sp_plc_tag.get())
                    tag.write_attribute(ua.AttributeIds.Value, ua.DataValue(float(sp)))

            if self.cv_write_value.get() != "":
                cv = float(self.cv_write_value.get())
                if self.comm_type == "logix":
                    ret = self.comm_write.Write(self.sp_plc_tag.get(), cv)
                    if ret.Status != "Success":
                        raise Exception(ret.TagName + " " + ret.Status)
                elif self.comm_type == "opc":
                    tag = self.comm_write.get_node(self.cv_plc_tag.get())
                    tag.write_attribute(ua.AttributeIds.Value, ua.DataValue(float(cv)))

        except Exception as e:
            self.gui_status.set("Write Error: " + str(e))

        else:
            self.gui_status.set("Data sent to PLC")

        finally:
            if self.comm_type == "logix":
                self.comm_write.Close()
            elif self.comm_type == "opc":
                self.comm_write.disconnect()

    def live_trend(self):
        """
        Set up a live trend plot using matplotlib, continuously updating
        with setpoint, process variable, and control variable data.
        """

        # Set up a live trend plot
        def init():
            if not 1 in plt.get_fignums():
                self._empty_plot_setup()

        def animate(i):
            with self.thread_lock:
                x = np.arange(len(self.SP)) / self.scale
                if len(x) == len(self.SP) == len(self.PV) == len(self.CV):
                    self.plot_SP.set_data(x, self.SP)
                    self.plot_CV.set_data(x, self.CV)
                    self.plot_PV.set_data(x, self.PV)
                    self.ax.relim()
                    self.ax.autoscale_view()

        self.anim = animation.FuncAnimation(self.fig, animate, init_func=init, frames=60, interval=1000)
        plt.show(block=False)

    def _on_plot_close(self, event):
        """
        Stop Animation when the Live Plot is closed.
        Enable Button to allow plot to be re-opened.
        """
        if self.anim:
            self.anim.event_source.stop()
            self.anim = None
        if self.looped_thread_for_data_retrieval:
            self.button_livetrend.configure(state=ttk.NORMAL)

    def show_live_trend(self):
        """
        Control the display of the live trend plot window.
        """
        self.button_livetrend.configure(state=ttk.DISABLED)
        plot_open = 1 in plt.get_fignums()
        if not plot_open:
            self._empty_plot_setup()
            self.live_trend()

    def open_trend_from_file(self):
        self.button_show_file_data.configure(state=ttk.DISABLED)
        thread = threading.Thread(target=self._open_trend_from_file_thread, name="Parse_File_Thread", daemon=True)
        thread.start()

    def _open_trend_from_file_thread(self):
        """
        Read data from a CSV file and plot setpoint, process variable, and control
        variable trends using matplotlib.
        """
        try:
            headers, timestamps, values = None, None, None
            if self.looped_thread_for_data_retrieval:
                self.csv_file.flush()

            # Read CSV file into a numpy array
            with open(self.file_name.get(), "r", encoding="utf-8") as historical_csv_file:
                csvreader = csv.reader(historical_csv_file, delimiter=";")
                headers = next(csvreader)
                data = np.array(list(csvreader))

            # Convert the timestamp column to datetime
            timestamps = np.array(data[:, -1], dtype="datetime64[ms]")
            # Convert other columns to float
            values = data[:, :-1].astype(float)

        except Exception as e:
            self.gui_status.set("CSV Read Error: " + str(e))

        finally:
            self.root.after(0, self._process_historical_plot, headers, timestamps, values)

    def _process_historical_plot(self, headers=None, timestamps=None, values=None):
        """
        Show plot
        """
        self.button_show_file_data.configure(state=ttk.NORMAL)

        if headers is not None and timestamps is not None and values is not None:
            # Set plot location on screen
            plt.figure(num=2)
            plt.clf()
            plt.gca().legend([])
            mngr = plt.get_current_fig_manager()
            mngr.window.geometry(f"{int(self.screen_width/2)}x{self.screen_height-self.toolbar}+{int(self.screen_width/2)-self.offset}+0")

            # Plot each column
            plt.plot(timestamps, values[:, 0], color="blue", linewidth=2, label=headers[0])
            plt.plot(timestamps, values[:, 1], color="red", linewidth=2, label=headers[1])
            plt.plot(timestamps, values[:, 2], color="darkgreen", linewidth=2, label=headers[2])

            # Customize plot
            plt.ylabel("Value")
            plt.xlabel("Time")
            plt.title(self.file_name.get())
            plt.legend(loc="upper right")
            plt.gcf().autofmt_xdate()

            # Show the plot
            plt.show(block=False)

    def _change_gui_state(self, req_state):
        """
        Modify the state (enabled or disabled) of GUI input elements.
        """
        self.entry_sp_tag.configure(state=req_state)
        self.entry_pv_tag.configure(state=req_state)
        self.entry_cv_tag.configure(state=req_state)
        self.entry_delta_t.configure(state=req_state)
        self.entry_plc_address.configure(state=req_state)
        self.entry_slot.configure(state=req_state)
        self.entry_file_name.configure(state=req_state)

    def stop(self):
        """
        Stop the data recording process, reset GUI elements, close files,
        disconnect from PLC/OPC server, and handle errors during shutdown.
        """
        try:
            # Stop data recording, reset GUI elements, and close files
            self.thread_stop_event.set()
            time.sleep(self.delta_t.get() / 1000)
            if self.looped_thread_for_data_retrieval:
                self.looped_thread_for_data_retrieval.stop()
                self.looped_thread_for_data_retrieval = None

            self._change_gui_state(req_state=ttk.NORMAL)
            self.button_livetrend.configure(state=ttk.DISABLED)
            self.button_start.configure(state=ttk.NORMAL)

            if self.csv_file:
                if not self.csv_file.closed:
                    self.csv_file.close()

            self.anim = None
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
    logger_app = PIDLogger()
    logger_app.root.mainloop()
