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
import threading
from typing import Callable, Tuple, List, Optional
from scipy.integrate import odeint
import numpy as np

INITIAL_MODEL_GAIN = 2.5
INITIAL_MODEL_TC = 75.5
INITIAL_MODEL_DT = 10.5
INITIAL_MODEL_BIAS = 13.5
INITIAL_KP = 1.9
INITIAL_KI = 0.1
INITIAL_KD = 1.1
INITIAL_LOGIX_IP_ADDRESS = "192.168.1.100"
INITIAL_LOGIX_SLOT = 2
INITIAL_OPC_ADDRESS = "opc.tcp://192.168.1.100:49320"
INITIAL_SP_TAG = "PID_SP"
INITIAL_PV_TAG = "PID_PV"
INITIAL_CV_TAG = "PID_CV"
MIN_PID_SIM_DURATION = 300
MAX_PID_SIM_DURATION = 18000


class PeriodicInterval(threading.Thread):
    """
    A class for running a task function periodically at a specified interval.

    This class inherits from threading.Thread and runs the specified task function
    periodically with a given period.

    Attributes:
        - task_function (Callable[[], None]): The task function to be executed periodically.
        - period (float): The period in seconds between consecutive executions of the task function.
        - i (int): A counter to keep track of the number of periods elapsed.
        - t0 (float): The starting time of the first period.
        - stop_event (threading.Event): An event to control stopping the thread.
        - locker (threading.Lock): A lock to ensure thread-safe execution of the task function.
    """

    def __init__(self, task_function: Callable[[], None], period: float) -> None:
        """
        Initialize the PeriodicInterval thread.
        """
        super().__init__()
        self.daemon = True
        self.task_function: Callable[[], None] = task_function
        self.period: float = period
        self.i: int = 0
        self.t0: float = time.time()
        self.stop_event: threading.Event = threading.Event()
        self.locker: threading.Lock = threading.Lock()
        self.start()

    def sleep(self) -> None:
        """
        Sleep for the remaining time to meet the specified period.
        """
        self.i += 1
        delta: float = self.t0 + self.period * self.i - time.time()
        if delta > 0:
            time.sleep(delta)

    def run(self) -> None:
        """
        Start the thread and execute the task_function periodically.
        """
        while not self.stop_event.is_set():
            with self.locker:
                self.task_function()
            self.sleep()

    def stop(self) -> None:
        """
        Set the stop event to terminate the periodic task execution.
        """
        self.stop_event.set()


class PIDController:
    """
    Proportional-Integral-Derivative (PID) controller.

    Attributes:
        - Kp (float): Proportional gain.
        - Ki (float): Integral gain.
        - Kd (float): Derivative gain.
        - setpoint (float): Target setpoint for the controller.
        - _min_output (float): Minimum allowed controller output value.
        - _max_output (float): Maximum allowed controller output value.
        - _proportional (float): Proportional term value.
        - _integral (float): Integral term value.
        - _derivative (float): Derivative term value.
        - output_limits (Tuple[float, float]): Tuple containing the minimum and maximum allowed controller output values.
        - _last_eD (float): The previous error value used for derivative calculation.
        - _lastCV (float): The previous controller output value.
        - _d_init (int): A flag to indicate whether the derivative term has been initialized.
    """

    def __init__(self) -> None:
        """
        Initialize the PID controller with default values.
        """
        self.Kp: float = 1
        self.Ki: float = 0.1
        self.Kd: float = 0.01
        self.setpoint: float = 50
        self._min_output: float = 0
        self._max_output: float = 100
        self._proportional: float = 0
        self._integral: float = 0
        self._derivative: float = 0
        self.output_limits: Tuple[float, float] = (0, 100)
        self._last_eD: float = 0
        self._lastCV: float = 0
        self._d_init: int = 0
        self.reset()

    def __call__(self, PV: float = 0, SP: float = 0, direction: str = "Direct") -> float:
        """
        Calculate the control value (CV) based on the process variable (PV) and the setpoint (SP).
        """
        # P term
        if direction == "Direct":
            e: float = SP - PV
        else:
            e = PV - SP
        self._proportional = self.Kp * e

        # I Term
        if 0 < self._lastCV < 100:
            self._integral += self.Ki * e
        # Allow I Term to change when Kp is set to Zero
        if self.Kp == 0 and self._lastCV == 100 and self.Ki * e < 0:
            self._integral += self.Ki * e
        if self.Kp == 0 and self._lastCV == 0 and self.Ki * e > 0:
            self._integral += self.Ki * e

        # D term
        eD: float = -PV
        self._derivative = self.Kd * (eD - self._last_eD)

        # init D term
        if self._d_init == 0:
            self._derivative = 0
            self._d_init = 1

        # Controller Output
        CV: float = self._proportional + self._integral + self._derivative
        CV = self._clamp(CV, self.output_limits)

        # update stored data for next iteration
        self._last_eD = eD
        self._lastCV = CV
        return CV

    @property
    def components(self) -> Tuple[float, float, float]:
        """
        Get the individual components of the controller output.
        """
        return self._proportional, self._integral, self._derivative

    @property
    def tunings(self) -> Tuple[float, float, float]:
        """
        Get the current PID tuning values (Kp, Ki, and Kd).
        """
        return self.Kp, self.Ki, self.Kd

    @tunings.setter
    def tunings(self, tunings: Tuple[float, float, float]) -> None:
        """
        Set new PID tuning values (Kp, Ki, and Kd).
        """
        self.Kp, self.Ki, self.Kd = tunings

    @property
    def output_limits(self) -> Tuple[float, float]:
        """
        Get the current output limits (minimum and maximum allowed controller output values).
        """
        return self._min_output, self._max_output

    @output_limits.setter
    def output_limits(self, limits: Optional[Tuple[float, float]]) -> None:
        """
        Set new output limits (minimum and maximum allowed controller output values).
        """
        if limits is None:
            self._min_output, self._max_output = 0, 100
            return
        min_output, max_output = limits
        self._min_output = min_output
        self._max_output = max_output
        self._integral = self._clamp(self._integral, self.output_limits)

    def reset(self) -> None:
        """
        Reset the controller values to their initial state.
        """
        self._proportional = 0
        self._integral = 0
        self._derivative = 0
        self._integral = self._clamp(self._integral, self.output_limits)
        self._last_eD = 0
        self._d_init = 0
        self._lastCV = 0

    def _clamp(self, value: float, limits: Tuple[float, float]) -> float:
        """
        Clamp the given value between the specified limits.
        """
        lower, upper = limits
        if value is None:
            return lower
        value = min(value, upper)
        value = max(value, lower)
        return value


class FOPDTModel:
    """
    First Order Plus Dead Time (FOPDT) Model.

    This class models a process with first-order dynamics and a time delay (dead time).

    Parameters:
        - Gain (float): The gain of the process.
        - Time_Constant (float): The time constant of the process.
        - Dead_Time (float): The dead time (time delay) of the process.
        - Bias (float): The bias value of the process.
        - stored_cv (List[float]): List to store control values over time.
    """

    def __init__(self) -> None:
        """
        Initialize the FOPDTModel with default parameters and an empty list for control values (CV).

        Attributes:
            - stored_cv (List[float]): List to store control values over time.
            - Gain (float): Gain of the process.
            - Time_Constant (float): Time constant of the process.
            - Dead_Time (float): Dead time (time delay) of the process.
            - Bias (float): Bias value of the process.
        """
        self.stored_cv: List[float] = []
        self.Gain: float = 1.0
        self.Time_Constant: float = 1.0
        self.Dead_Time: float = 1.0
        self.Bias: float = 0.0

    def change_params(self, data: Tuple[float, float, float, float]) -> None:
        """
        Update the model parameters with new values.
        """
        self.Gain, self.Time_Constant, self.Dead_Time, self.Bias = data

    def _calc(self, init_pv: float, ts: float) -> float:
        """
        Calculate the change in the process variable (PV) over time.
        """
        if (ts - self.Dead_Time) <= 0:
            um: float = 0
        elif int(ts - self.Dead_Time) >= len(self.stored_cv):
            um = self.stored_cv[-1]
        else:
            um = self.stored_cv[int(ts - self.Dead_Time)]
        dydt: float = (-(init_pv - self.Bias) + self.Gain * um) / self.Time_Constant
        return dydt

    def update(self, init_pv: float, ts: List[float]) -> float:
        """
        Update the process variable (PV) using the FOPDT model.
        """
        y: np.ndarray = odeint(self._calc, init_pv, ts)
        return float(y[-1])


def convert_gains_for_sim(gains, pid_form="independent", pid_time_units="seconds") -> tuple:
    """
    Convert the PID Gains to be used in simulation.

    Parameters:
    - gains (tuple): A tuple containing the original PID gains (Kp, Ki, Kd).
    - pid_form (str): A string indicating the form of PID controller ('independent' or 'dependent').
    - time_units (str): A string indicating the time units ('seconds' or 'minutes').

    Returns:
    - tuple: A tuple containing the converted PID gains (Kp, Ki, Kd) appropriate for the simulation.
    """
    calc_p, calc_i, calc_d = gains
    if pid_form == "independent" and pid_time_units == "seconds":
        return gains

    elif pid_form == "independent" and pid_time_units == "minutes":
        kp_sec = calc_p
        ki_sec = calc_i / 60.0
        kd_sec = calc_d * 60.0
        return (kp_sec, ki_sec, kd_sec)

    elif pid_form == "dependent" and pid_time_units == "seconds":
        kp_sec = calc_p
        ki_sec = calc_p / calc_i if calc_i != 0 else 0.0
        kd_sec = calc_d * calc_p
        return (kp_sec, ki_sec, kd_sec)

    elif pid_form == "dependent" and pid_time_units == "minutes":
        kp_sec = calc_p
        ki_sec = (calc_p / calc_i if calc_i != 0 else 0.0) / 60
        kd_sec = (calc_d * calc_p) * 60
        return (kp_sec, ki_sec, kd_sec)


def calculate_pid_response(model_params, tune_values) -> tuple:
    """
    Simulate the PID response.

    Parameters:
    - model_params (tuple): A tuple containing the process model parameters (model_gain, model_tc, model_dt, model_bias).
    - tune_values (tuple): A tuple containing the PID tuning values (Kp, Ki, Kd).

    Returns:
    - tuple: A tuple containing arrays of the simulation results and ITAE value.
        - sim_sp_array (np.array): The setpoint values array.
        - sim_cv_array (np.array): The control variable values array.
        - sim_pv_array (np.array): The process variable values array.
        - p_term_array (np.array): The proportional term values array.
        - i_term_array (np.array): The integral term values array.
        - d_term_array (np.array): The derivative term values array.
        - itae (float): The Integral of Time-weighted Absolute Error (ITAE) value.
    """
    # Unpack
    model_gain, model_tc, model_dt, model_bias = model_params
    kp, ki, kd = tune_values

    # Find the size of the range needed
    calc_duration = int(model_dt * 2 + model_tc * 5)
    sim_length = min(max(calc_duration, MIN_PID_SIM_DURATION), MAX_PID_SIM_DURATION - 1)

    # Setup arrays
    sim_sp_array = np.zeros(sim_length)
    sim_pv_array = np.zeros(sim_length)
    sim_cv_array = np.zeros(sim_length)
    p_term_array = np.zeros(sim_length)
    i_term_array = np.zeros(sim_length)
    d_term_array = np.zeros(sim_length)
    np.random.seed(0)
    noise = np.random.uniform(-0.2, 0.2, sim_length)

    process_model = FOPDTModel()
    pid = PIDController()
    process_model.change_params((model_gain, model_tc, model_dt, model_bias))
    process_model.stored_cv = sim_cv_array

    # Defaults
    start_of_step = 10
    direction = "Direct" if model_gain > 0 else "Reverse"

    # Update Process model
    sim_gains = convert_gains_for_sim(gains=(kp, ki, kd))
    # Get PID ready
    pid.tunings = sim_gains
    pid.reset()
    # Set initial value
    sim_pv_array[0] = model_bias + noise[0]

    # Loop through timestamps
    for i in range(sim_length - 1):
        # Adjust the Setpoint
        if i < start_of_step:
            sim_sp_array[i] = model_bias
        elif direction == "Direct":
            sim_sp_array[i] = 60 + model_bias if i < sim_length * 0.6 else 40 + model_bias
        else:
            sim_sp_array[i] = -60 + model_bias if i < sim_length * 0.6 else -40 + model_bias
        # Find current controller output
        sim_cv_array[i] = pid(sim_pv_array[i], sim_sp_array[i], direction)
        # Find calculated sim_pv_array
        sim_pv_array[i + 1] = process_model.update(sim_pv_array[i], [i, i + 1])
        sim_pv_array[i + 1] += noise[i]
        # Store individual terms
        p_term_array[i], i_term_array[i], d_term_array[i] = pid.components
        # Calculate Integral Time weighted Average of the Error (ITAE)
        itae = 0 if i < start_of_step else itae + (i - start_of_step) * abs(sim_sp_array[i] - sim_pv_array[i])

    return sim_sp_array[:-1], sim_cv_array[:-1], sim_pv_array[:-1], p_term_array[:-1], i_term_array[:-1], d_term_array[:-1], itae / sim_length


def find_pid_gains(model_params) -> dict[str, dict[str, float]]:
    """
    Calculate the PID gains using different tuning methods.

    Parameters:
    - model_params (tuple): A tuple containing the process model parameters (model_gain, model_tc, model_dt, model_bias).

    Returns:
    - dict: A dictionary containing PID gains for different tuning methods.
        - 'chr': Gains using the CHR tuning method:
            - 'chr_kp': Proportional gain (float).
            - 'chr_ki': Integral gain (float).
            - 'chr_kd': Derivative gain (float).
        - 'imc': Gains using the IMC tuning method:
            - 'imc_kp': Proportional gain (float).
            - 'imc_ki': Integral gain (float).
            - 'imc_kd': Derivative gain (float).
        - 'aimc': Gains using the AIMC tuning method:
            - 'aimc_kp': Proportional gain (float).
            - 'aimc_ki': Integral gain (float).
            - 'aimc_kd': Derivative gain (float).
    """
    model_gain, model_tc, model_dt, model_bias = model_params
    if model_tc <= 0:
        model_tc = 0.1
    if model_dt <= 0:
        model_dt = 0.1
    # CHR Kp
    num = 0.6 * model_tc
    den = abs(model_gain) * model_dt
    chr_kp = num / den
    # CHR Ki
    ti = 1 * model_tc
    chr_ki = chr_kp / ti
    # CHR Kd
    td = 0.5 * model_dt
    chr_kd = chr_kp * td

    # IMC Kp
    lmda = 2.1 * model_dt
    num = model_tc + 0.5 * model_dt
    den = abs(model_gain) * (lmda)
    imc_kp = 1.1 * (num / den)
    # IMC Ki
    ti = model_tc + 0.5 * model_dt
    imc_ki = imc_kp / ti
    # IMC Kd
    num = model_tc * model_dt
    den = 2 * model_tc + model_dt
    td = num / den
    imc_kd = 1.1 * (td * imc_kp)

    # AIMC Kp
    L = max(0.1 * model_tc, 0.8 * model_dt)
    num = model_tc + 0.5 * model_dt
    den = abs(model_gain) * (L + 0.5 * model_dt)
    aimc_kp = num / den
    # AIMC Ki
    ti = model_tc + 0.5 * model_dt
    aimc_ki = aimc_kp / ti
    # AIMC Kd
    num = model_tc * model_dt
    den = 2 * model_tc + model_dt
    td = num / den
    aimc_kd = td * aimc_kp

    return {
        "chr": {"kp": round(chr_kp, 4), "ki": round(chr_ki, 4), "kd": round(chr_kd, 4)},
        "imc": {"kp": round(imc_kp, 4), "ki": round(imc_ki, 4), "kd": round(imc_kd, 4)},
        "aimc": {"kp": round(aimc_kp, 4), "ki": round(aimc_ki, 4), "kd": round(aimc_kd, 4)},
    }


def find_pi_gains(model_params) -> dict[str, dict[str, float]]:
    """
    Calculate the PID gains using different tuning methods.

    Parameters:
    - model_params (tuple): A tuple containing the process model parameters (model_gain, model_tc, model_dt, model_bias).

    Returns:
    - dict: A dictionary containing PID gains for different tuning methods.
        - 'chr': Gains using the CHR tuning method:
            - 'chr_kp': Proportional gain (float).
            - 'chr_ki': Integral gain (float).
            - 'chr_kd': Derivative gain (float).
        - 'imc': Gains using the IMC tuning method:
            - 'imc_kp': Proportional gain (float).
            - 'imc_ki': Integral gain (float).
            - 'imc_kd': Derivative gain (float).
        - 'aimc': Gains using the AIMC tuning method:
            - 'aimc_kp': Proportional gain (float).
            - 'aimc_ki': Integral gain (float).
            - 'aimc_kd': Derivative gain (float).
    """
    model_gain, model_tc, model_dt, model_bias = model_params
    if model_tc <= 0:
        model_tc = 0.1
    if model_dt <= 0:
        model_dt = 0.1
    # CHR Kp
    num = 0.35 * model_tc
    den = abs(model_gain) * model_dt
    chr_kp = num / den
    # CHR Ki
    ti = 1.2 * model_tc
    chr_ki = chr_kp / ti
    # CHR Kd
    chr_kd = 0

    # IMC Kp
    lmda = 2.1 * model_dt
    num = model_tc + 0.5 * model_dt
    den = abs(model_gain) * (lmda)
    imc_kp = num / den
    # IMC Ki
    ti = model_tc + 0.5 * model_dt
    imc_ki = imc_kp / ti
    # IMC Kd
    imc_kd = 0

    # AIMC Kp
    L = max(0.1 * model_tc, 0.8 * model_dt)
    aimc_kp = model_tc / (abs(model_gain) * (model_dt + L))
    # AIMC Ki
    ti = model_tc / (1.03 - 0.165 * (model_dt / model_tc))
    aimc_ki = aimc_kp / ti
    # AIMC Kd
    aimc_kd = 0

    return {
        "chr": {"kp": round(chr_kp, 4), "ki": round(chr_ki, 4), "kd": round(chr_kd, 4)},
        "imc": {"kp": round(imc_kp, 4), "ki": round(imc_ki, 4), "kd": round(imc_kd, 4)},
        "aimc": {"kp": round(aimc_kp, 4), "ki": round(aimc_ki, 4), "kd": round(aimc_kd, 4)},
    }
