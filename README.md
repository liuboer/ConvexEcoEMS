# ConvexEcoEMS
Code for the paper [Convex Optimization for Long-term Eco-driving of Fuel Cell Hybrid Electric Vehicles on Signalized Corridors](http://dx.doi.org/10.1109/TVT.2024.3443106) published in ***IEEE Transactions on Vehicular Technology***.

## Description

Eco-driving for Fuel Cell Hybrid Electric Vehicles (FCHEVs) through signalized intersections is a coupled problem of speed planning and powertrain control under complex environmental constraints. For global optimality and fast computation, this paper proposes a spatially convex long-term eco-driving approach for FCHEVs on signalized corridors. Considering road slopes, speed limits, and traffic lights, the original eco-driving problem is reformulated as a convex second-order cone programming problem by pre-optimization of the best green light window and convex approximation and convex relaxation of the hybrid powertrain. A powertrain-aware green window planner is first used to determine the optimal passing time windows through signalized intersections. Then the convex eco-driving problem is formulated and finally solved by concurrent optimization and sequential optimization according to whether the speed planning problem and energy management problem are coupled. Results show that the proposed concurrent convex optimization algorithm performs better fuel economy than the sequential optimization algorithm with similar computational time and can reduce motor energy consumption by 4.25% compared to an analytical speed planner. Compared to dynamic programming-based concurrent optimization, the proposed eco-driving method achieves 93.45% fuel economy with only 0.80% computational time.

## Open source code used

1. [dpm.m](https://idsc.ethz.ch/research-guzzella-onder/downloads.html) is a dynamic programming Matlab function.

2. Code for the GER-MILP algorithm is based on [Optimal Routing for Hybrid Vehicles](https://git.rwth-aachen.de/avt-svt/public/optimal-routing), whose details can be found in [Optimal Eco-Routing for Hybrid Vehicles With Powertrain Model Embedded](https://doi.org/10.1109/TITS.2021.3131298).
