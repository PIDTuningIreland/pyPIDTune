# pyPIDTune - Python PID Tuner

![PyPI](https://img.shields.io/pypi/v/pypidtune?label=pypi%20package)
![PyPI - Downloads](https://img.shields.io/pypi/dm/pypidtune)
![PyPI - Python Version](https://img.shields.io/pypi/pyversions/pypidtune)
![GitHub repo size](https://img.shields.io/github/repo-size/PIDTuningIreland/pyPIDTune)
![PyPI - License](https://img.shields.io/pypi/l/pypidtune)


To install:

```
pip install pypidtune
```


PID tuning in 4 Steps:
```
A-> Record a PRC using the PID Logger
B-> Tune using the PID Tuner
C-> Refine the tune using PID Simulator
D-> Test the tune with FOPDT Process Emulator
```


# **PID Logger**

To launch, use:
```
import pypidtune

pid_logger = pypidtune.PIDLogger()
pid_logger.root.mainloop()
   
```


![Logger](https://raw.githubusercontent.com/PIDTuningIreland/pyPIDTune/main/screenshots/logger.png)



# **PID Simulator**

To launch, use:
```
import pypidtune

pid_simulator = pypidtune.PIDSimulator()
pid_simulator.root.mainloop()
    
```


![Simulator](https://raw.githubusercontent.com/PIDTuningIreland/pyPIDTune/main/screenshots/simulator.png)



# **Process Emulator**

To launch, use:
```
import pypidtune

pid_emulator = pypidtune.ProcessEmulator()
pid_emulator.root.mainloop()
    
```


![Emulator](https://raw.githubusercontent.com/PIDTuningIreland/pyPIDTune/main/screenshots/emulator.png)


---


# **PID Tuner**

To launch, use:
```
import pypidtune

pid_tuner = pypidtune.PIDTuner()
pid_tuner.root.mainloop()
    
```


![Tuner](https://raw.githubusercontent.com/PIDTuningIreland/pyPIDTune/main/screenshots/tuner.png)


---

# **beoTune© Adapative Auto-Tuner**



https://github.com/user-attachments/assets/46a9ba28-b4ca-4fb7-ad83-43ef1c42f0e6


```
Premium Feature - https://github.com/sponsors/PIDTuningIreland
```
