Discoveries
---
1. There exists and inverse relationship between number of best trajectories chosen to evaluate and the the resolution of x and theta velocity samples.
If (x, theta) resolution is set to (0.05, 0.05) then 3 best trajectories chosen is sufficient.
If (x, theta) resolution is set to (0.01, 0.01) then 8 best trajectories chosen is sufficient.
This is due to the fact that at higher resolution most paths are similar so we need to analyse a larger of number of paths so that we cang get sufficiently different number of paths.


Decrease resolution and increase span area at every step
increase sim time at every step
Add cost scoring from center of obstacle
add buffers for best trajectories, the buffers will be randomly sample and will be away from best trajectories and have less simlarity to the best trajectories