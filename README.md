# ikd-Tree-ros

ikd-tree implementation on ROS by using my own pcd file.


**ikd-Tree** is an incremental k-d tree designed for robotic applications. The ikd-Tree incrementally updates a k-d tree with new coming points only, leading to much lower computation time than existing static k-d trees. Besides point-wise operations, the ikd-Tree supports several features such as box-wise operations and down-sampling that are practically useful in robotic applications.

But I noticed that in static situations, conventional FLANN-based search provided in PCL is more faster.

Please check the `pcd_demo.cpp`

## How to run

```
catkin build ikd_tree_ros
rosrun ikd_tree_ros pcd_demo
```


Then, given 4,505,471 of cloud points, below commands are printed (note that former case denotes the time taken by ikd tree and latter case for the time taken by FLANN kd tree in PCL).
```
# of pts: 4505471
Multi thread started
2.30039sec. vs 1.16074 sec.
8.417 vs 3.22
```

