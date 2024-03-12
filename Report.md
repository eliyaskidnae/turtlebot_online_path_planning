# Online Path Planner lab1

### Group Members 
   1. Eliyas Kidanemariam Abraha - u1992469
   2. Goitom Leaku 

## Table of Contents
1. [Introduction](#introduction)
2. [Implementation](#features)
3. [Installation](#installation)
4. [Usage](#usage)
5. [Graph ](#contributing)
6. [Demo Video](#license)

## Introduction
The lab integrated concepts from occupancy grid mapping and path planning to navigate the robot from its current position to a goal position while avoiding obstacles in the environment. This report presents an overview of the project, its architecture, implementation details, and the delivered package.

## Implementation

### State Validity Checker

The State Validity Checker module was responsible for verifying whether a given configuration (position) was valid or not. It implemented three key functions: set, isValid, and checkPath. The set function updated the map using the received occupancy grid data, while isValid checked if a specific position was free of obstacles. The checkPath function ensured the validity of the planned path, especially important when dynamic obstacles were present.

In our simulation with the TurtleBot3 Waffle Model, we've tailored our approach to ensure comprehensive coverage while navigating through environments. To account for the dimensions of the TurtleBot3 Waffle (approximately 0.29m x 0.31m x 0.15m in length, width, and height respectively), we've set the distance parameter to 0.2 meters. This configuration enables the robot to survey a rectangular area around its current position, spanning 0.4 meters in both length and width directions, providing a sufficiently broad scope for assessing path validity. A distance of 0.2 meters allows the robot to navigate through relatively narrow spaces such as doorways while still conducting thorough validity checks within its immediate vicinity. Increasing this parameter could potentially impede the robot's ability to navigate through narrow passages, limiting its agility in complex environments.

When **validating paths**, we've adopted a discretization strategy that aligns with our objectives. To ensure seamless path validation without overlooking critical details, we've employed a step size equal to the distance parameter (0.2 meters). This choice guarantees that successive paths are scrutinized with precision, minimizing the risk of overlooking potential obstacles or hazards.By keeping the step size the same as our distance parameter, we ensure our path checks are reliable. If we made the step size smaller, we'd look at more area, but it would take longer. So, our current way works well for checking paths thoroughly without slowing us down too much.

The following figures shows difference step size with thier obstacel checking area.

<div style="display: flex; justify-content: center;">
    <div style="flex: 1; margin-right: 10px;">
        <img src="./imgs/distance_0.4.png" alt="Figure 1" width="250"/>
        <p style="text-align: center;">Figure 1: With step size = 2*distance</p>
    </div>
    <div style="flex: 1; margin-right: 10px;">
        <img src="./imgs/distance_0.2.png" alt="Figure 2" width="250"/>
        <p style="text-align: center;">Figure 2: with step size = distance</p>
    </div>
    <div style="flex: 1;">
        <img src="./imgs/distance_0.1.png" alt="Figure 3" width="250"/>
        <p style="text-align: center;">Figure 3: With step size = 0.5*distance</p>
    </div>
</div>


### RRT Planner 
  Rapidly-exploring Random Tree (RRT) is a sampling-based motion planning algorithm used to efficiently explore high-dimensional spaces. It is particularly well-suited for problems involving complex, high-dimensional configuration spaces.

  In the provided implementation, the main class used is RRT, which represents the Rapidly-exploring Random Tree algorithm. This class is responsible for generating a feasible path from the start to the goal configuration in a given environment while avoiding obstacles.We create rrt_module.py inside the /utils_lib folder inorder to modulerize the planning module.which is the main part of this lab.
  the rrt_modle has two class:
  - **Node Class** - The Node class represents a node in the RRT tree. Each node has attributes such as x and y coordinates, an id to uniquely identify it, f_score and g_score used in path planning algorithms and a parent pointer to track the parent node in the tree.This class provides methods to calculate heuristic values (calcu_huristic), calculate distances between nodes (get_distance), find the nearest node (find_nearest_node), and filter nodes within a given radius (find_nodes_with_in_radius). Additionally, it defines a __str__ method to represent the node as a string.

  - **RRT Class** - The RRT class represents the Rapidly-exploring Random Tree algorithm. It initializes with parameters such as the environment (svc), number of iterations (k), step size (q), probability of selecting the goal (p), domain of the space (dominion), maximum time for computation (max_time), and whether to use RRT* algorithm (is_RRT_star).The class contains methods to find the optimal parent node within a given radius (cost_optimal_parent), rewire the tree to improve paths (rewire), generate a random configuration within the domain (Rand_Config), find the nearest vertex (Near_Vertices), generate a new configuration (New_Config), reconstruct the path from start to goal (reconstract_path),and get the tree as a list of edges (get_tree). 
  
  The *compute_path* method is the main function that generates a feasible path from the start to the goal configuration using the RRT algorithm. It iteratively samples random configurations, connects them to the tree, and updates the tree structure until it reaches the goal or completes the specified number of iterations.
    
  **Path Smoothing** : A smooth path is a simplified trajectory that removes unnecessary or jagged movements while maintaining the overall path from the start to the goal. It involves iteratively examining consecutive points along the path and removing any that can be skipped without significantly altering the trajectory. This process reduces the number of waypoints, resulting in simpler and more efficient execution by the robot or vehicle. The path smoothing process is implemented in the smooth_path method within the RRT class, where unnecessary waypoints are removed by checking if a straight line segment between consecutive points avoids obstacles.


if whether to use RRT* algorithm (is_RRT_star) is enabled the following functionality of the RRT* algorithm are also included:

1. **Rewiring**: 
   - The rewire functionality in the RRT algorithm aims to improve the efficiency of the tree by considering potential alternative paths to the new node. It does so by examining nearby nodes within a certain radius and checking if the newly added node provides a shorter path to them compared to their current parent node.
   
2. **cost function**:
   - In the RRT algorithm, the cost function is used to determine the optimal parent node for a newly added node within a certain radius. The goal of the cost function is to select the parent node that minimizes the overall cost of reaching the new node from the tree's root node while considering the constraints of the problem, such as collision avoidance.
   
### Controller  
The controller continuously adjusts the velocity commands sent to the robot based on its current state and the desired goal, allowing it to navigate effectively towards the target position and orientation while considering factors such as distance, orientation, and obstacles. Proportional control is employed to adjust the control signals (linear and angular velocities) based on the errors. Using the controller, the robot's linear and angular velocities are determined based on the distance remaining to reach the next waypoint and the desired orientation. The control gains, Kv and Kw, determine how the robot should move linearly and angularly. When the controller is applied to the robot, the orientation is controlled before distance, to prevent sudden shifts in orientation that may cause the robot to deviate from the planned path if the linear part is corrected first. As long as the distance from the current pose to the next target pose is higher than the tolerance (0.2 in our case), the robot's linear and angular velocities are controlled by Kv and Kw, respectively.

v_d  = Kv * d, where *d* is the distance between the current pose and the next waypoint the robot is moving towards.

w_d = Kw * θ, where *θ* is the orientation error to the desired pose of the robot

We include logic to handle special cases or conditions, such as when the orientation error exceeds a certain threshold, 0.1 radians(5.8 degrees). In such cases, the controller may adjust the control signals to ensure stable and efficient operation.It stops the linear velocity (v_d = 0) and increases the angular velocity (w_d).
This is to facilitate turning towards the goal direction more quickly, when the threshold is so small the robot moves in a correct way as it adjusts at every step when this small angle is happen. So we set a threshold value of o.1 radians so the robot adjusts its orientation as long as the orientaion error is higher than the threshold. 


### To Do Implementations and Other in Ros Node 

1. When Goal Positition is given from rvis first we check if the goal position is valid , if it is not valid we can't plan and we send message to give valid goal poition.
2. If the current position of the robot is not valid we try to recover the robot to valid position by publishing velocity command to move to any around valid pose.
3. If the  current position and goal position is valid we initiate path planning  using RRT path planner.
4. If Path is given from planner we publish velocity command to move the robot to the next waypoint while checking the path is valid path.Beacause we are getting new map from octamap server previously unkown cells may be obstacles we should have to check every time we recive map if  the path is correct.if the path is not valid  we initiate new plan to RRT.
5. Checking path and Replanning is done only if we are considering unkown pose as valid pose.
4. If a path from rrt is empty which means the planner can't get a path to the goal point due to different reasons we create a counter for how many times to we should have to try to get a path from planner.if path is not found in that trial we assume there is no valid path betteewen start point and goal point.
6. beside the  smoothed path  we also add another marker to draw the tree of the rrt edges this helps us to visulaize everting.

Below two figures visualizes the RRT tree which is in blue and the 
e smoothed path is also shown in red color.
<div style="display: flex; justify-content: center;">
    <div style="flex: 1;">
        <img src="./imgs/rrt7_1.png" alt="Figure 1" width="400"/>
        <p style="text-align: center;">Figure 4: RRT Tree and smoothed path </p>
    </div>
    <div style="flex: 1;">
        <img src="./imgs/rrt8_1.png" alt="Figure 2" width="400"/>
        <p style="text-align: center;">Figure 5: RRT Tree and smoothed path </p>
    </div>

</div>

Watch a demo of our project in action:

<video width="640" height="360" controls>
  <source src="./imgs/demo1.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
<video width="640" height="360" controls>
  <source src="./imgs/demo2.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
<video width="640" height="360" controls>
  <source src="./imgs/demo3.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
List and describe the key features of the project, such as:
- Types of images and animations included
- Interactivity or user engagement features
- Compatibility with different platforms or devices

## Installation
Describe how to install or set up the project to view or interact with the images and animations. Include any necessary dependencies, software, or hardware requirements.

## Usage
Provide instructions on how to use or interact with the images and animations. This might include:
- How to open or access the images/animations
- How to navigate through the content
- Any controls or options available to the user

## Contributing
Outline guidelines for contributing to the project, such as how others can submit new images/animations, suggest improvements, or report issues.

## License
Specify the license under which the project is distributed. This could be an open-source license like MIT or GPL, or a custom license. Include any terms or conditions for using or distributing the images/animations.

## Additional Sections (Optional)
Depending on the nature of your project, you may want to include additional sections such as:

- Examples: Showcase example images or animations included in the project.
- Customization: Provide instructions on how users can customize or modify the images/animations.
- Resources: List any additional resources or references related to images, animations, or the project topic.
- Credits: Acknowledge individuals or organizations who contributed images, animations, or other assets to the project.
- Support: Offer contact information or links to support channels for users who have questions or issues related to the project.

## Screenshots/GIFs (Optional)
Include screenshots or GIFs to visually demonstrate the images/animations included in the project. This can help users understand what to expect and encourage them to explore further.

## Troubleshooting (Optional)
Provide solutions to common issues or errors that users may encounter while accessing or viewing the images/animations.

## Version History (Optional)
Keep track of changes or updates made to the project over time. Include version numbers, dates, and summaries of changes.

## Feedback
Encourage users to provide feedback on the project, whether it's through GitHub issues, email, or other channels.

## Conclusion
Conclude with any final thoughts, recommendations, or next steps related to the project.
