# Multi Path Trajectory Tracking in Dynamic Traffic Environment

## Overview
This repository contains the codebase for the "Multi Path Trajectory Tracking in Dynamic Traffic Environment" project developed during the "Constrained Numerical Optimization for Estimation and Control" course. The project aims to develop an optimal control system for autonomously navigating a vehicle through traffic on a multi-lane road, focusing on safety and performance objectives.

## Project Structure
The repository structure is organized as follows:

- **main.m**: Main script orchestrating the entire project.

- **plantModelInit.m**: Script for initializing the plant model.
  
- **SimPlanner/SimPlannerInit.m**: Script for initializing the simulation enviroment and parameters. 

- **deliverables**: Contains various project deliverables, including presentations, reports, and images.
  
- **optimizer**: Includes files related to optimization algorithms, such as gradient descent and constrained optimization.

- **SimPlanner**: Contains scripts for simulation planning and management.

- **envVisualization.m**: Script for visualizing the environment.


## Assumptions
- **Road Geometry:** The road is assumed to be straight without curves.
- **Lane Configuration:** The road consists of three lanes plus an emergency lane to their right.
- **Vehicle Consideration:** Only the three closest vehicles to the ego vehicle are considered in optimization procedures.
- **Constant Speed:** Other vehicles are assumed to be proceeding at a constant speed.
- **Plant Model:** The plant model is assumed to perfectly match reality and is consistent throughout the control strategy.
- **Data Acquisition:** All required quantities are assumed to be available from lower-level systems within the vehicle.

## Simulation Environment
The simulation environment block serves as a digital twin of the working environment and the main vehicle itself. It includes subblocks for the vehicle model, simulation planner, and environment graphics, facilitating testing and evaluation of the control system.

## Control System
The control system is based on Model Predictive Control (MPC) and is developed around the single-track model of the ego vehicle. It receives inputs regarding desired normal running conditions, compares them with measurements from the external world, and generates control actions to maintain safe running conditions.

## Model
The main model is the single-track model of the ego vehicle, along with constant speed dynamics models of the three nearest vehicles

## Video

https://github.com/MarioBertelli/CNOECProject2023/assets/32597982/115a447e-2146-4e9e-a9ec-5c7201221d0b


https://github.com/MarioBertelli/CNOECProject2023/assets/32597982/3868dc2a-bfb5-4161-ab32-ce018d573d25




https://github.com/MarioBertelli/CNOECProject2023/assets/32597982/0bcf8c0d-6092-4bd2-a93b-cd58955f9883




https://github.com/MarioBertelli/CNOECProject2023/assets/32597982/4c6db2d7-d2f4-4960-95e9-4bd6dd38400f

