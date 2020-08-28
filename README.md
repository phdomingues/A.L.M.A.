# A.L.M.A.

A.L.M.A. = Autonomous Light and Multi-Assistant

TCC group in automation and control engineering:

	- Pedro Henrique Silva Domingues: Computer vision / image processing task;
	- Lucas Meirelles Carregaro: Kinematic model implementation
	- Ângelo Jorge Bálsamo: Kinematic modeling

Presentation:
https://youtu.be/JzujezFcufQ

Objective:
	Make a robotic arm capable of assisting dentists on the consultory by tracking the patient face and keep it illuminated without the interference of the dentist.

- Premisses:

	1. dentist is wearing a mask full time. (help avoiding confusion when the face is lost by thinking the dentist is the patient)
	2. dentist is wearing white or black silicone gloves full time.
	
Requirements:

	- Ubuntu 16.04
	- Python 2.7
	- Python 3
	- ROS Kinetic
	- Rviz
	- Movit

Python 3 dependencies:

	- csv
	- numpy
	- scikit-learn
	
Python 2.7 dependencies:
	
	- csv
	- dlib
	- numpy
	- opencv
