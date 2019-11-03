# Tello-pilot


## Drone Controller
This is an implementation of Autopilot for Tello Drone.
The base library used for operating the tello drone is implemented by [damiafuntes](https://github.com/damiafuentes/DJITelloPy)
I have made some minor changes to this library to make it more autonomous and robust to be implemented for an Autopilot.

# Autopilot
1. To run the autopilot you need to have your laptop wifi connected to the drone wifi. once connected you are ready to go.
2. Use the following command to run the program. 
    ```shell
    python tello_fast_game.py
    ```
    
# Implementation Details
The ability of this autopilot is limited to following a person or a face depending upon the mode specified. 
Once a face or a person is found in the view the autopilot used various algorithms to track the object (KCF tracker).
The person in the image is detected by using the "YOLO V3" neural network.
