# Stillsuit
The Stillsuit, a novel lower-limb cable-driven exosuit with a state-of-the-art transmission efficiency, is controlled via a hierarchical controller as shown below. 

![image](https://github.com/user-attachments/assets/8b2015e2-3b98-4c3c-9a75-f39351a9b826)

It uses the ```PyCandle``` library to communicate via CAN with its AK-60 motors and ```board``` to communicate via i2c with the IMUs and load cells.

- The **supervisory control** (sometimes called Finite State Machine) extracts the IMU and load cell readings to detect whether the user is in stance and the time at which to apply a given torque profile.
- The **high-level controller** sets the desired torque profile to apply during stance phase. It is currently set to a standard square wave, but can be tuned to the participant with optimisation algorithms such as Bayesian Optimisation or CMA-ES. 
- The **low-level controller** carries out the desired torque profile sent by the high-level controller, through torque PI control.

A pre-tensioning mechanism is implemented through velocity PID control to remove any slack in the system at the beginning of every walking experiment.

More information coming soon.
