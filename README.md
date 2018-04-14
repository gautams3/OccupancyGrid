# OccupancyGrid
## Background
As part of my work with the MathWorks Robotics System Toolbox (RST), I built a prototype for a probabilistic occupancy grid, when we were expanding our features for implementing SLAM using the RST. Here are some slides on grid maps for robotics.

This is a prototype I created as part of a project, where I implemented:

* Binary Occupancy Grids
* Ternary Occupancy Grids (occupied, unoccupied, or unknown)
* Probabilistic Occupancy Grids with [log-odds probability](http://www.statisticshowto.com/log-odds/), and
* Probabilistic Occupancy Grids with hit-and-miss probability. 

It included a [laser-beam sensor model](https://people.eecs.berkeley.edu/~pabbeel/cs287-fa11/slides/beam-sensor-model.pdf) for a range sensor. Below is a video link of a lidar that scans the environment once to get a probabilistic occupancy grid of the space around it. The probability of occupancy goes from 0 (white) to 1 (black).

[![Watch the video](https://img.youtube.com/vi/IUvRzk35SeM/0.jpg)](https://youtu.be/IUvRzk35SeM)

(For more details, see [this post](https://www.gautamsalhotra.com/2015/11/probabilistic-occupancy-grid.html) on [my site](https://www.gautamsalhotra.com))

## How to run
Run main.m in MATLAB. It might take some time but will eventually show you
  * a binary occupancy grid
  * a ternary occupancy grid
  * a probabilistic occupancy grid
  * a figure of log-odds probability
  * a video (MoccGrid.avi) that shows how the occupancy grid is updated as the scans from the sensor are processed
