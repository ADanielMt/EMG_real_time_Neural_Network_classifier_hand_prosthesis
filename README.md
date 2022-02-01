# CONTROL OF ACTIVE UPPER LIMB PROSTHESIS USING ARTIFICIAL INTELLIGENCE AND FORCE FEEDBACK

This repository contains the files used to control an active robotic hand prosthesis using EMG signals.

The signals are acquired using an EMG MYO Armband from Thalmic Labs and a Raspberry Pi 3B+. The data acquired
is stored and feature extraction is performed. Then, real time classification of hand movements and grasps is 
done using a pre-trained neural network. The result of the classification is codified and sent to a master 
microcontroler named "MicroRDR" (ATMega328P), which decodes the result and sends a signal to two slave 
microcontrollers, MicroPLM and MicroIMA. Those microcontrollers contains specific routines to control the fingers
of an active robotic hand prosthesis. The microcontrollers also receive signals of support sensors (Motors angular
positions, Force Sensing Resistors - FSR, Limit Switch). Using this information, the microcontrollers controls 
the motors so the robotic hand prosthesis actively reachs the desired position (the hand movement classified 
by the Neural Network Model).
