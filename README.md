# NavCon

NavCon is a modular framework for natural language guided navigation for mobile robots. For more information please see our paper

[Tell Me Where to Go: A Composable Framework for Context-Aware Embodied Robot Navigation](https://openreview.net/forum?id=fviZhMCr62)


NavCon uses a server (NavConServer) to process visual inputs from a mobile robot. The server is based on [ViperGPT](https://github.com/cvlab-columbia/viper) and we would like to thank the authors for their work. NavConServer takes in visual inputs from the robot and a natural language sentence. Code is then generated using a foundation model (GPT-3.5, or GPT-4). The output is a bounding box that can be used for robot navigation.

NavCon client runs on the robot and natural language requests can be made to the server using a rosservice call such as:

```rosservice call nav_server "sentence: 'Go to the chair.'```

This call will send images to the server for processing and the result will be a 2D center coordinate that can be used for robot navigation. The coordinate can be converted to a 3D waypoint using the marble_mapping package. A projection is performed on the input Octomap based map and from there the 3D coordinate is passed into a graph based planner (```scan_plan```) for robot navigation. The bobcat module acts as mission manager for the planner. Path following can be performed using ```marble_guidance``` For more information on the navigation stack please see:
[Flexible Supervised Autonomy for Exploration in Subterranean Environments.](https://doi.org/10.55417/fr.2023004)

More details on setting up the server and client can be found in the READMEs of each respective package. Setting up the navigation stack is highly dependent on your robotic setup. Generally, ```LIO-SAM```  ([LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping](https://github.com/TixiaoShan/LIO-SAM)) needs to be configured to accept inputs from your LiDAR and IMU which will provide state estimates for the robot. The estimate and lidar point cloud can then be used in ```marble_mapping``` to generate 3D volumetric maps. From there the maps can be sent into ```scan_plan``` to perform graph based path planning. Marble guidance uses the robot odometry and path plan to move the robot while bobcat controls the state (exploration, going to a waypoint, returning to the start, etc) of the planner.


If you find this work useful please cite:

```@article{biggie2023tell,
  title={Tell Me Where to Go: A Composable Framework for Context-Aware Embodied Robot Navigation},
  author={Biggie, Harel and Mopidevi, Ajay Narasimha and Woods, Dusty and Heckman, Christoffer},
  journal={arXiv preprint arXiv:2306.09523},
  year={2023}
}
```
