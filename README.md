# ChanceConstrained-ELM-Learning
Implementation of a discrete-time dynamical system model learning method from demonstration while providing probabilistic guarantees on the safety and stability of the learned model.

The code is written in MATLAB 2020b and requires the CVX package with an academic MOSEK license to solve the optimization problem.

To install CVX with MOSEK, follow the instruction given in http://cvxr.com/cvx/download/

Three examples are provided to implement the learning method. 

1 - A two-link robot planar manipulator

2 - A mobile wheeled robot 

3 - A human hand-drawn shape using the LASA publicly available dataset (https://www.epfl.ch/labs/lasa/datasets/)

Each example is under its respective folder, and to run the simulation, "main.m" needs to be executed. The design parameters for each simulation example can be adjusted inside "main.m" to fine-tune according to the state trajectories to be learned. 
