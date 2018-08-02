# Cognitive Automobile Laboratory
This is an excellent [course project](https://www.mrt.kit.edu/lehre_SS_Kognitive_Automobile_Labor.php) from [MRT](https://www.mrt.kit.edu/index.php) of Karlsruhe Institute of Technology, focusing on implementation of algorithms for autonomous driving.<br/><br/>
<p align="center">
  <a href="https://www.youtube.com/watch?v=ryiSJ79cOZs&t=7s"><img src="https://i.imgur.com/lSv9e1u.png" width="70%" height="70%" title="Final Competition - Click to Watch"/></a>
</p>
<br/>

## Team KITAF
- Perception Team: Kuangye Chen, Esteban Rivera
- Control Team: Weiyun Chen, Tianjing Chen
<p align="center">
  <img src="https://i.imgur.com/i1QlKhR.jpg" width="50%" height="50%" title="Traffic Sign Detection Demo - Click to Watch"/>
</p>

## Important Packages from Team KITAF

### kitaf_detector_ros_tool
Detection of traffic signs and other vehicles based on TensorFlow and [SSD](https://arxiv.org/pdf/1512.02325.pdf) neural network.<br/>
Our result is pretty impressive, which is capable of detecting traffic signs in real time with an accuracy above 96%.<br/>
<p align="center">
  <a href="https://www.youtube.com/watch?v=TLpxAJpOwIY"><img src="https://j.gifs.com/yrNmz7.gif" title="Traffic Sign Detection Demo - Click to Watch"/></a>
</p>

### kitaf_lateral_control_ros_tool
Controlling the vehicle based on [stanley controller](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf).<br/>
We first use kinematic model for its easy implementation, and then move to dynamic model for better performance.

### map_creation_ros_tool, kitaf_navigation_ros_tool
- Generating an [occupancy grid](https://docs.ufpr.br/~danielsantos/ProbabilisticRobotics.pdf) map based on point cloud from kinect2. With local grid maps from each time step, we maintain a global grid map with bayes updating rule.
- Providing path when driving along the road. Use [potential field](https://www.cs.mcgill.ca/~hsafad/robotics/) method based on occupancy grid to plan a path for going throungh the unknown environment with pylons.
<p align="center">
  <a href="https://www.youtube.com/watch?v=pCMyGIkVGqE"><img src="https://j.gifs.com/E9kPgm.gif" title="Path Planning Demo - Click to Watch"/></a>
</p>

### kitaf_following_ros_tool
Control our vehicle to follow another vehicle based on detection results and point clouds.
<p align="center">
  <a href="https://www.youtube.com/watch?v=Tp6dlefkCds"><img src="https://j.gifs.com/rRDYrW.gif" title="Path Planning Demo - Click to Watch"/></a>
</p>

## Final Competition
We won the first place in the final! 🏆
