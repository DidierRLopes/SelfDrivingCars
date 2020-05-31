SELF-DRIVING CARS SPECIALIZATION
==============

Be at the forefront of the autonomous driving industry. With market researchers predicting a $42-billion market and more than 20 million self-driving cars on the road by 2025, the next big job boom is right around the corner.

This Specialization gives you a comprehensive understanding of state-of-the-art engineering practices used in the self-driving car industry. You'll get to interact with real data sets from an autonomous vehicle (AV)―all through hands-on projects using the open source simulator CARLA.

Throughout your courses, you’ll hear from industry experts who work at companies like Oxbotica and Zoox as they share insights about autonomous technology and how that is powering job growth within the field.

You’ll learn from a highly realistic driving environment that features 3D pedestrian modelling and environmental conditions. When you complete the Specialization successfully, you’ll be able to build your own self-driving software stack and be ready to apply for jobs in the autonomous vehicle industry.

It is recommended that you have some background in linear algebra, probability, statistics, calculus, physics, control theory, and Python programming. You will need these specifications in order to effectively run the CARLA simulator: Windows 7 64-bit (or later) or Ubuntu 16.04 (or later), Quad-core Intel or AMD processor (2.5 GHz or faster), NVIDIA GeForce 470 GTX or AMD Radeon 6870 HD series card or higher, 8 GB RAM, and OpenGL 3 or greater (for Linux computers).

You’ll learn from a highly realistic driving environment that features 3D pedestrian modelling and environmental conditions. When you complete the Specialization successfully, you’ll be able to build your own self-driving software stack and be ready to apply for jobs in the autonomous vehicle industry.

# Introduction to Self-Driving Cars

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

    
    
# State Estimation and Localization for Self-Driving Cars

Welcome to State Estimation and Localization for Self-Driving Cars, the second course in University of Toronto’s Self-Driving Cars Specialization. We recommend you take the first course in the Specialization prior to taking this course. 

This course will introduce you to the different sensors and how we can use them for state estimation and localization in a self-driving car. By the end of this course, you will be able to:
- Understand the key methods for parameter and state estimation used for autonomous driving, such as the method of least-squares
- Develop a model for typical vehicle localization sensors, including GPS and IMUs
- Apply extended and unscented Kalman Filters to a vehicle state estimation problem
- Understand LIDAR scan matching and the Iterative Closest Point algorithm 
- Apply these tools to fuse multiple sensor streams into a single state estimate for a self-driving car 

For the final project in this course, you will implement the Error-State Extended Kalman Filter (ES-EKF) to localize a vehicle using data from the CARLA simulator. 

This is an advanced course, intended for learners with a background in mechanical engineering, computer and electrical engineering, or robotics. To succeed in this course, you should have programming experience in Python 3.0, familiarity with Linear Algebra (matrices, vectors, matrix multiplication, rank, Eigenvalues and vectors and inverses), Statistics (Gaussian probability distributions), Calculus and Physics (forces, moments, inertia, Newton's Laws).

## Module 0: Welcome to Course 2: State Estimation and Localization for Self-Driving Cars
This module introduces you to the main concepts discussed in the course and presents the layout of the course. The module describes and motivates the problems of state estimation and localization for self-driving cars.

## Module 1: Least Squares
The method of least squares, developed by Carl Friedrich Gauss in 1795, is a well known technique for estimating parameter values from data. This module provides a review of least squares, for the cases of unweighted and weighted observations. There is a deep connection between least squares and maximum likelihood estimators (when the observations are considered to be Gaussian random variables) and this connection is established and explained. Finally, the module develops a technique to transform the traditional 'batch' least squares estimator to a recursive form, suitable for online, real-time estimation applications.
	
## Module 2: State Estimation - Linear and Nonlinear Kalman Filters
Any engineer working on autonomous vehicles must understand the Kalman filter, first described in a paper by Rudolf Kalman in 1960. The filter has been recognized as one of the top 10 algorithms of the 20th century, is implemented in software that runs on your smartphone and on modern jet aircraft, and was crucial to enabling the Apollo spacecraft to reach the moon. This module derives the Kalman filter equations from a least squares perspective, for linear systems. The module also examines why the Kalman filter is the best linear unbiased estimator (that is, it is optimal in the linear case). The Kalman filter, as originally published, is a linear algorithm; however, all systems in practice are nonlinear to some degree. Shortly after the Kalman filter was developed, it was extended to nonlinear systems, resulting in an algorithm now called the ‘extended’ Kalman filter, or EKF. The EKF is the ‘bread and butter’ of state estimators, and should be in every engineer’s toolbox. This module explains how the EKF operates (i.e., through linearization) and discusses its relationship to the original Kalman filter. The module also provides an overview of the unscented Kalman filter, a more recently developed and very popular member of the Kalman filter family.    

## Module 3: GNSS/INS Sensing for Pose Estimation
To navigate reliably, autonomous vehicles require an estimate of their pose (position and orientation) in the world (and on the road) at all times. Much like for modern aircraft, this information can be derived from a combination of GPS measurements and inertial navigation system (INS) data. This module introduces sensor models for inertial measurement units and GPS (and, more broadly, GNSS) receivers; performance and noise characteristics are reviewed. The module describes ways in which the two sensor systems can be used in combination to provide accurate and robust vehicle pose estimates.

## Module 4: LIDAR Sensing
LIDAR (light detection and ranging) sensing is an enabling technology for self-driving vehicles. LIDAR sensors can ‘see’ farther than cameras and are able to provide accurate range information. This module develops a basic LIDAR sensor model and explores how LIDAR data can be used to produce point clouds (collections of 3D points in a specific reference frame). Learners will examine ways in which two LIDAR point clouds can be registered, or aligned, in order to determine how the pose of the vehicle has changed with time (i.e., the transformation between two local reference frames).

## Module 5: Putting It together - An Autonomous Vehicle State Estimator
This module combines materials from Modules 1-4 together, with the goal of developing a full vehicle state estimator. Learners will build, using data from the CARLA simulator, an error-state extended Kalman filter-based estimator that incorporates GPS, IMU, and LIDAR measurements to determine the vehicle position and orientation on the road at a high update rate. There will be an opportunity to observe what happens to the quality of the state estimate when one or more of the sensors either 'drop out' or are disabled.



# Visual Perception for Self-Driving Cars

Welcome to Visual Perception for Self-Driving Cars, the third course in University of Toronto’s Self-Driving Cars Specialization.

This course will introduce you to the main perception tasks in autonomous driving, static and dynamic object detection, and will survey common computer vision methods for robotic perception.  By the end of this course, you will be able to work with the pinhole camera model, perform intrinsic and extrinsic camera calibration, detect, describe and match image features and design your own convolutional neural networks.  You'll apply these methods to visual odometry, object detection and tracking, and semantic segmentation for drivable surface estimation. These techniques represent the main building blocks of the perception system for self-driving cars.

For the final project in this course, you will develop algorithms that identify bounding boxes for objects in the scene, and define the boundaries of the drivable surface.  You'll work with synthetic and real image data, and evaluate your performance on a realistic dataset.

This is an advanced course, intended for learners with a background in computer vision and deep learning. To succeed in this course, you should have programming experience in Python 3.0, and familiarity with Linear Algebra (matrices, vectors, matrix multiplication, rank, Eigenvalues and vectors and inverses).

## Module 0: Welcome to Course 3: Visual Perception for Self-Driving Cars
This module introduces the main concepts from the broad field of computer vision needed to progress through perception methods for self-driving vehicles. The main components include camera models and their calibration, monocular and stereo vision, projective geometry, and convolution operations.

## Module 1: Basics of 3D Computer Vision
This module introduces the main concepts from the broad field of computer vision needed to progress through perception methods for self-driving vehicles. The main components include camera models and their calibration, monocular and stereo vision, projective geometry, and convolution operations.

## Module 2: Visual Features - Detection, Description and Matching
Visual features are used to track motion through an environment and to recognize places in a map. This module describes how features can be detected and tracked through a sequence of images and fused with other sources for localization as described in Course 2. Feature extraction is also fundamental to object detection and semantic segmentation in deep networks, and this module introduces some of the feature detection methods employed in that context as well.

## Module 3: Feedforward Neural Networks
Deep learning is a core enabling technology for self-driving perception. This module briefly introduces the core concepts employed in modern convolutional neural networks, with an emphasis on methods that have been proven to be effective for tasks such as object detection and semantic segmentation. Basic network architectures, common components and helpful tools for constructing and training networks are described.

## Module 4: 2D Object Detection
The two most prevalent applications of deep neural networks to self-driving are object detection, including pedestrian, cyclists and vehicles, and semantic segmentation, which associates image pixels with useful labels such as sign, light, curb, road, vehicle etc. This module presents baseline techniques for object detection and the following module introduce semantic segmentation, both of which can be used to create a complete self-driving car perception pipeline.

## Module 5: Semantic Segmentation
The second most prevalent application of deep neural networks to self-driving is semantic segmentation, which associates image pixels with useful labels such as sign, light, curb, road, vehicle etc. The main use for segmentation is to identify the drivable surface, which aids in ground plane estimation, object detection and lane boundary assessment. Segmentation labels are also being directly integrated into object detection as pixel masks, for static objects such as signs, lights and lanes, and moving objects such cars, trucks, bicycles and pedestrians.

## Module 6: Putting it together - Perception of dynamic objects in the drivable region
The final module of this course focuses on the implementation of a collision warning system that alerts a self-driving car about the position and category of obstacles present in their lane. The project is comprised of three major segments: 1) Estimating the drivable space in 3D, 2) Semantic Lane Estimation and 3) Filter wrong output from object detection using semantic segmentation.



# Motion Planning for Self-Driving Cars

Welcome to Motion Planning for Self-Driving Cars, the fourth course in University of Toronto’s Self-Driving Cars Specialization.

This course will introduce you to the main planning tasks in autonomous driving, including mission planning, behavior planning and local planning.   By the end of this course, you will be able to find the shortest path over a graph or road network using Dijkstra's and the A* algorithm, use finite state machines to select safe behaviors to execute, and design optimal, smooth paths and velocity profiles to navigate safely around obstacles while obeying traffic laws.  You'll also build occupancy grid maps of static elements in the environment and learn how to use them for efficient collision checking. This course will give you the ability to construct a full self-driving planning solution, to take you from home to work while behaving like a typical driving and keeping the vehicle safe at all times.

For the final project in this course, you will implement a hierarchical motion planner to navigate through a sequence of scenarios in the CARLA simulator, including avoiding a vehicle parked in your lane, following a lead vehicle and safely navigating an intersection.  You'll face real-world randomness and need to work to ensure your solution is robust to changes in the environment.

This is an intermediate course, intended for learners with some background in robotics, and it builds on the models and controllers devised in Course 1 of this specialization. To succeed in this course, you should have programming experience in Python 3.0, and familiarity with Linear Algebra (matrices, vectors, matrix multiplication, rank, Eigenvalues and vectors and inverses) and calculus (ordinary differential equations, integration).

## Welcome to Course 4: Motion Planning for Self-Driving Cars
This module introduces the motion planning course, as well as some supplementary materials.

## Module 1: The Planning Problem
This module introduces the richness and challenges of the self-driving motion planning problem, demonstrating a working example that will be built toward throughout this course. The focus will be on defining the primary scenarios encountered in driving, types of loss functions and constraints that affect planning, as well as a common decomposition of the planning problem into behaviour and trajectory planning subproblems. This module introduces a generic, hierarchical motion planning optimization formulation that is further expanded and implemented throughout the subsequent modules.

## Module 2: Mapping for Planning
The occupancy grid is a discretization of space into fixed-sized cells, each of which contains a probability that it is occupied. It is a basic data structure used throughout robotics and an alternative to storing full point clouds. This module introduces the occupancy grid and reviews the space and computation requirements of the data structure. In many cases, a 2D occupancy grid is sufficient; learners will examine ways to efficiently compress and filter 3D LIDAR scans to form 2D maps.

## Module 3: Mission Planning in Driving Environments
This module develops the concepts of shortest path search on graphs in order to find a sequence of road segments in a driving map that will navigate a vehicle from a current location to a destination. The modules covers the definition of a roadmap graph with road segments, intersections and travel times, and presents Dijkstra’s and A* search for identification of the shortest path across the road network.

## Module 4: Dynamic Object Interactions
This module introduces dynamic obstacles into the behaviour planning problem, and presents learners with the tools to assess the time to collision of vehicles and pedestrians in the environment.

## Module 5: Principles of Behaviour Planning
This module develops a basic rule-based behaviour planning system, which performs high level decision making of driving behaviours such as lane changes, passing of parked cars and progress through intersections. The module defines a consistent set of rules that are evaluated to select preferred vehicle behaviours that restrict the set of possible paths and speed profiles to be explored in lower level planning.

## Module 6: Reactive Planning in Static Environments
A reactive planner takes local information available within a sensor footprint and a global objective defined in a map coordinate frame to identify a locally feasible path to follow that is collision free and makes progress to a goal. In this module, learners will develop a trajectory rollout and dynamic window planner, which enables path finding in arbitrary static 2D environments. The limits of the approach for true self-driving will also be discussed.

## Module 7: Putting it all together - Smooth Local Planning
Parameterized curves are widely used to define paths through the environment for self-driving. This module introduces continuous curve path optimization as a two point boundary value problem which minimized deviation from a desired path while satisfying curvature constraints.
