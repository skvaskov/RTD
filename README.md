# RTD Examples: Segway and Rover

This repository contains code for the paper, ["Bridging the Gap Between Safety and Real-Time Performance in Receding-Horizon Trajectory Design for Mobile Robots"](https://arxiv.org/abs/1809.06746), which introduces **Reachability-based Trajectory Design**, or RTD.

Check out the [tutorial](https://github.com/skousik/RTD_tutorial) to get a gentler introduction to the method. This repository follows the tutorial, but for the Segway and Rover instead of for the Turtlebot.

<img src="figures/segway_time_lapse.jpg" alt="segway_time_lapse" width="600" />



#### Getting Started

All the code in this repository runs in MATLAB (R2018a or newer).

This repository contains almost everything you need to start playing with RTD. We have included a "frozen in time" version of our [simulator](https://github.com/skousik/simulator) repository.

To run the reachable set computations, you'll need [MOSEK\(https://www.mosek.com/) (free for academic use).

To run the NMPC planner in the comparison code, you'll need to get [GPOPS](http://www.gpops2.com/) (free for academic use).

#### Citing this Work

Please cite the [paper](https://arxiv.org/abs/1809.06746). Also, check out our other papers: [RTD on a car](https://arxiv.org/abs/1902.01786), [RTD on a drone](https://arxiv.org/abs/1904.05728), and [RTD on an arm](https://arxiv.org/abs/2002.01591).

#### Authors

Sean Vaskov, Shreyas Kousik, Hannah Larson, and Ram Vasudevan
