Introduction to Self-Driving Cars
==============

Welcome to Introduction to Self-Driving Cars, the first course in University of Toronto’s Self-Driving Cars Specialization.

This course will introduce you to the terminology, design considerations and safety assessment of self-driving cars.  By the end of this course, you will be able to:
    * Understand commonly used hardware used for self-driving cars
    * Identify the main components of the self-driving software stack
    * Program vehicle modelling and control
    * Analyze the safety frameworks and current industry practices for vehicle development

For the final project in this course, you will develop control code to navigate a self-driving car around a racetrack in the CARLA simulation environment. You will construct longitudinal and lateral dynamic models for a vehicle and create controllers that regulate speed and path tracking performance using Python. You’ll test the limits of your control design and learn the challenges inherent in driving at the limit of vehicle performance.

This is an advanced course, intended for learners with a background in mechanical engineering, computer and electrical engineering, or robotics. To succeed in this course, you should have programming experience in Python 3.0, familiarity with Linear Algebra (matrices, vectors, matrix multiplication, rank, Eigenvalues and vectors and inverses), Statistics (Gaussian probability distributions), Calculus and Physics (forces, moments, inertia, Newton's Laws).

You will also need certain hardware and software specifications in order to effectively run the CARLA simulator: Windows 7 64-bit (or later) or Ubuntu 16.04 (or later), Quad-core Intel or AMD processor (2.5 GHz or faster), NVIDIA GeForce 470 GTX or AMD Radeon 6870 HD series card or higher, 8 GB RAM, and OpenGL 3 or greater (for Linux computers).

## Module 0: Welcome to the Self-Driving Cars Specialization!
This module will introduce you to the main concepts and layout of the specialization and discusses the major advances made in the field over the last two decades, highlighting the most recent progress made by major players in terms of safety and performance metrics, where available.

## Module 1: The Requirements for Autonomy
Self-driving cars present an extremely rich and inter-disciplinary problem. This module introduces the language and structure of the problem definition, defining the most salient elements of the driving task and the driving environment.

## Module 2: Self-Driving Hardware and Software Architectures
System architectures for self-driving vehicles are extremely diverse, as no standardized solution has yet emerged. This module describes both the hardware and software architectures commonly used and some of the tradeoffs in terms of cost, reliability, performance and complexity that constrain autonomous vehicle design.

## Module 3: Safety Assurance for Autonomous Vehicles
As the self-driving domain matures, the requirement for safety assurance on public roads become more critical to self-driving developers. You will evaluate the challenges and approaches employed to date to tackle the immense challenge of assuring the safe operation of autonomous vehicles in an uncontrolled public road driving environment.

## Module 4: Vehicle Dynamic Modeling
The first task for automating an driverless vehicle is to define a model for how the vehicle moves given steering, throttle and brake commands. This module progresses through a sequence of increasing fidelity physics-based models that are used to design vehicle controllers and motion planners that adhere to the limits of vehicle capabilities.

## Module 5: Vehicle Longitudinal Control
Longitudinal control of an autonomous vehicle involves tracking a speed profile along a fixed path, and can be achieved with reasonable accuracy using classic control techniques. This week, you will learn how to develop a baseline controller that is applicable for a significant subset of driving conditions, which include most non-evasive or highly-dynamic motions.

## Module 6: Vehicle Lateral Control
This week, you will learn about how lateral vehicle control ensures that a fixed path through the environment is tracked accurately. You will see how to define geometry of the path following control problem and develop both a simple geometric control and a dynamic model predictive control approach.

## Module 7: Putting it all together
For the last week of the course, now you will get hands on with a simulation of an autonomous vehicle that requires longitudinal and lateral vehicle control design to track a predefined path along a racetrack with a given speed profile. You are encouraged to modify the speed profile and/or path to improve their lap time, without any requirement to do so. Work and play!
