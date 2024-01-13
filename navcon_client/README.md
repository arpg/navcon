# NavCon Client
This is a ros node client for the NavCon server. 

For a single images use:

`navcon_client.launch`

For three images use:

`navcon_client_three.launch`

You will need to update the arguments to match the setup both the setup of the server and your robot.

## Build NavCon Client

First step is to source your ROS.
```bash
# By defualt, ROS is located at
source /opt/ros/noetic/setup.bash
```

Make sure you have below packages installed. If not, you can
install using command (ROS noetic as example):
```bash
sudo apt-get update
sudo apt-get install -y \
    ros-noetic-cv-bridge \
    ros-noetic-pcl-conversions \
    ros-noetic-octomap-ros \
    ros-noetic-pcl-ros \
    ros-noetic-image-geometry \
    ros-noetic-dynamic-edt-3d
```

Then, install dependencies using ``rosdep`` in the workspace.
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Only for ROS noetic:

You will meet belowing error when installing dependencies. The issue is becuase the commit of NavCon uses
has some issues on ros noetic.
```bash
ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
lio_sam: Cannot locate rosdep definition for [GTSAM]
Continuing to install resolvable dependencies...
#All required rosdeps installed successfully
```

To resolve this, according to [issue_link](https://github.com/TixiaoShan/LIO-SAM/issues/206) we need to first install GTSAM 4.x from [official website](https://gtsam.org/get_started/).
I'm installing from Ubuntu PPA.
```bash
#Add PPA for the latest GTSAM 4.x stable release

# Add PPA
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update  # not necessary since Bionic
# Install:
sudo apt install libgtsam-dev libgtsam-unstable-dev

```
After installing, you can verif installation to see if it's installed under ``/usr/lib``
```bash
dpkg -L libgtsam-dev
```
After confirming installation, we need to add this path to ``CMAKE_PREFIX``
```bash
export CMAKE_PREFIX_PATH=/usr/lib/:$CMAKE_PREFIX_PATH
```
Then you need to configure ``include/utility.h`` in ``LIO-SAM`` folder. replace ``#include <opencv/cv.h>``
with ``#include <opencv2/opencv.hpp>``.

We also need to configure ``CMakeLists.txt`` to replace ``set(CMAKE_CXX_FLAGS "-std=c++11")`` with
 ``set(CMAKE_CXX_FLAGS "-std=c++14")``

> [!NOTE]
> According to the ``issue`` on lio_sam repo, one addition step was mentioned.
> ``Move #include <opencv2/opencv.hpp> after the pcl headers`` However, in this specific 
> commit using by NavCon, we don't have to perform this step.

Now, we should be able to build the whole Navcon package.

> [!NOTE]info
> To verify we have all depencies for ``lio_sam``, we can use ``catkin_make --pkg lio_sam``

### Build the Ros Package

Use ``catkin_make -j20 -l20`` or just ``catkin_make`` to build the workspace. 
``-j20 -l20`` is used by ``rough_octomap``. Hence, ``catkin_make build`` is not working here.
```bash
catkin_make
```

## Post installation

Now, we can source the NavCon client into our ros
```bash
source devel/setup.bash
```

We can now try launch the client by a .launch config.
```bash
roslaunch navcon_client navcon_client.launch
```

A similar output will be like below
```bash
NODES
  /
    scan_task_pub (rostopic/rostopic)
  /robot/
    navigation_client (navcon_client/navcon_navigation_node.py)
    project_coordinates (navcon_client/project_coordinates_node)

auto-starting new master
process[master]: started with pid [12480]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to b62ad64e-b19e-11ee-a057-103d1c680943
process[rosout-1]: started with pid [12509]
started core service [/rosout]
process[robot/project_coordinates-2]: started with pid [12516]
process[robot/navigation_client-3]: started with pid [12517]
process[scan_task_pub-4]: started with pid [12518]
[ INFO] [1705100571.866328638]: [Project Coordinates] Initializing Node.
[DEBUG] [1705100571.871172225]: [Project Coordinates]: Node Handle nampspace: /robot
[DEBUG] [1705100571.875654081]: [Project Coordinates:] Subscribing to cam_front/camera_info topic
[DEBUG] [1705100571.876371898]: [Project Coordinates:] Subscribing to cam_left/camera_info topic
[DEBUG] [1705100571.877732492]: [Project Coordinates:] Subscribing to cam_right/camera_info topic
[ INFO] [1705100571.882952289]: [Project Coordinates] Read Parameters

```
