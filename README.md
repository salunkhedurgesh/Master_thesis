# Master_thesis
Optimisation of a 2-dof orientation parallel mechanism

Master Thesis at Ecole Centrale de Nantes and LS2N \
Supervisors: Prof. Damien Chablat, Prof. Marcello Sanguineti, Dr. Guillaume Michel and Dr. Shivesh Kumar

# Aim of the thesis:
The thesis aims to contribute an analytical perspective into designing a 2 dof orientation parallel mechanism in order to manipulate an endoscope in an otological surgery. The beginning of the thesis started with singularity analysis and kinematic analysis and later proceeded onto optimization of the parameters. 

The report of the same can be found on: https://drive.google.com/file/d/1fjpB2BsJuB12C11z3jZiHe2A0ZXEp3ss/view?usp=sharing

# Optimization of the mechanism

Without discussing the pre-existing knowledge on the kinematic analysis of the mechanisms, lets directly get into some interesting aspect of the manipulators. When you optimize anything, the main components of the problem are:
1. Objective function Eg: max(x^2 - 2*x + 3)
2. Constraints Eg: such that: abs(x^3 - 3 + 6*sin(x) < 1)
3. Limits on the optimizing variables Eg: for x ranging from [-6, 7]

In case of manipulator the 2nd and 3rd part are easy. You solve inverse kinematics and you have the equation for your constraints. The workspace boundary forms the third part of the above mentioned optimization problem. But, defining the first part is rather challenging and interesting problem. Why? Because, imagine that you are interested in finding the maximum possible feasible workspace that is:
1. Non singular
2. Respects actuator limits
3. Respect passive joint limits
4. Avoids internal collisions
5. Is compact or has certain favourable proportions of parameters.

Try defining all of this in an equation. Maybe, its easy for some mechanisms but to accomodate the wildest imaginations of the revered designers of mechanism and their creative forte, we need to come up with a methodology that allows us to optimize a mechanism if we can define the 2nd part and 3rd part and have a clear idea of objective function (though we cannot define it in explicit form yet). For this we can use 'type O' optimization methods. These methods do not need the derivative of the function to find its maxima and minima and so do not care about the smoothnes, differentiability of the curve we are trying to optimize. One of such algorithms is the well known genetic algorithms. But, we will be implementing a much faster optimizing technique, Nelder Mead algorithm. The disadvantage of this algorithm is that we may get locked in local minimas and so to get around this we use multi start technique by extracting the initial points for each optimisation method by a low-discrepancy distribution of the set of points in our valid ranges (the 3rd part).

# Variations of the mechanism
While writing this README (15 April, 2020), I have uploaded 3 folders:
1. Nelder_Mead_2UPS
2. Nelder_Mead_2PUS_horizontal
3. Nelder_Mead_2PUS-Vertical

The mechanism in discussion is the one that implements the passive leg that defines the constraints of the mechanism. The 2UPS legs have 6 dof and thus can be arranged to have no reciprocals (sorry, if you don't know screw theory) and do not put any constraint on the mechanism. You can find the detailed information in my report mentioned above, the mechanism is shown in Figure: 1.4 of the report. The mechanism without its parallel bar extension is quite famous and widely known in the mechanism community. (One of the examples: Kinematic Analysis of a Novel Parallel 2SPRR+1U Ankle Mechanism in Humanoid Robot, Shivesh Kumar et. al)

The arrangement of 6 joints can be done in different fashion. UPS is one option while PUS is another. Both these configurations further yields two configurations where the Prismatic joint can be placed vertical or in horizontal position.

This yields different optimized results also highlighting the bad configurations. For example, the PUS configuration with prismatic joint in horizontal direction is super bad and should not be used if we can deploy any other configuration of the mechanism.

# Nelder_Mead_Fusion
As of 18th April, 2020. This folder is the important one. The aim for the codes in this folder is to be able to have the flexibility to implement any mechanism for Nelder Mead optimisation.
What do we need out of a new mechanism?
1. Inverse kinematics and the configuration space, especially those joint values on which the constraints is to be applied
2. Determinant of the jacobian matrix J in J.x_dot = q_dot..... Note that it is the reverse of generally known x_dot = J.q_dot.

That's it and we are happy to find the optimised solution.

It is super flexible. Write the objective function you want, define your own reward system to bias certain favourable parameters.
The constraints can be added or removed easily so it can help us with experimenting with different constraints.
