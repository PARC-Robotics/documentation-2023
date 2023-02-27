# Setting up your workspace with Docker on Windows

This guide will walk you through the process of setting up a ROS workspace with Docker, which will allow you to easily develop and test your ROS projects in a containerized environment. We will also cover how to connect VS Code to the Docker container and how to use X11 to run GUI Docker apps.

## Prerequisites
Before starting with the steps below, ensure that you have the following:

* Docker Desktop for Windows installed and running
* Visual Studio Code (VS Code) installed

## Install VcXsrv

1. Download and install [VcXsrv](https://sourceforge.net/projects/vcxsrv/){:target=_blank} on your computer.
2. Install VcXsrv by running the installer and following the prompts.
3. Once the installation is complete, open the VcXsrv by clicking on the Start menu and typing `XLaunch`.
4. In the XLaunch window, select the `Multiple windows` option and click `Next`.
    ![XLaunch step 1](assets/vcxsrv-install-1.png)
5. In the next window, select the `Start no client` option and click `Next`.
    ![XLaunch step 2](assets/vcxsrv-install-2.png)
6. In the next window, select the `Clipboard` option and click `Next`. Deselect the `Native opengl` option and select the `Disable access control` option.
    ![XLaunch step 3](assets/vcxsrv-install-3.png)
7. In the next window, click `Finish`.
    ![XLaunch step 4](assets/vcxsrv-install-4.png)

## Creating the main folder and `docker-compose.yml`

1. Create a new folder in your preferred location and name it whatever you like.
2. Inside the newly created folder, create a ROS workspace by running the following command in powershell:

    ```shell
    mkdir catkin_ws/src
    ```

3. Next, create a `docker-compose.yml` file inside the main folder. This file will contain the configuration for our ROS Docker container. Open the file in your favorite editor and add the following lines:

    ```yaml
    version: '3'
    services:
      ros:
        image: osrf/ros:noetic-desktop-full
        environment:
          - DISPLAY=host.docker.internal:0.0
          - ROS_HOSTNAME=ros
          - ROS_MASTER_URI=http://ros:11311
        volumes:
          - ./catkin_ws:/catkin_ws
        ports:
          - "11311:11311"
        command: roscore
    ```
    This configuration will pull the ROS Noetic Desktop Full image, if it does not already exist and configure it for X11 server support. It will also mount the `catkin_ws` folder inside the container and start the `roscore` command when the container starts.

4. Save the `docker-compose.yml` file and close your editor.

## Building the Docker container

1. Open a new terminal window and navigate to the main folder you created earlier.

2. Run the following command to build the Docker container:

    ```shell
    docker compose up -d
    ```

3. Once the container is built, you can verify that it's running by running the following command:

    ```shell
    docker ps
    ```

    You should see the following output:

    ```shell
    CONTAINER ID   IMAGE                          COMMAND                  CREATED          STATUS          PORTS                    NAMES
    db7df0798d9b   osrf/ros:noetic-desktop-full   "/ros_entrypoint.sh …"   20 seconds ago   Up 19 seconds    0.0.0.0:11311->11311/tcp   parc-ros-docker-ros-1
    ```

## Opening a terminal in the Docker container

1. To open a terminal in the Docker container, run the following command:

    ```shell
    docker exec -it parc-ros-docker-ros-1 bash
    ```
    where `parc-ros-docker-ros-1` is the name of the container. You can find the name of the container by running the `docker ps` command.

2. Once the terminal is open, you can verify that you are in the container by running the following command:

    ```shell
    echo $ROS_DISTRO
    ```

    You should see the following output:

    ```shell
    noetic
    ```

## Setting up the ROS workspace

1. Source the ROS environment:

    ```shell
    source /opt/ros/noetic/setup.bash
    ```

2. Navigate to the `catkin_ws` folder:

    ```shell
    cd /catkin_ws
    mkdir src
    cd src
    catkin_init_workspace
    ```

3. Initialize the workspace:

    ```shell
    cd ..
    catkin_make
    ```

4. Source the workspace:

    ```shell
    source devel/setup.bash
    ```

5. Configure bash to automatically source the workspace:

    ```shell
    echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    ```

## Clone the repository

In the same terminal (or in a new one), copy and paste the following:
```sh
cd ~/catkin_ws/src
git clone --recurse-submodules https://github.com/PARC-Robotics/PARC-Engineers-League.git
```
Or if you already have cloned the repo without submodules, run command `git submodule update --init --recursive` to update them.

## Install dependencies

In the same terminal (or in a new one), copy and paste the following:
```sh
cd ~/catkin_ws
sudo apt update
rosdep install --from-paths ./src --ignore-src -y
```

## Compile packages
```sh
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```


**NOTE:** There is a known issue while compiling, ` Intel RealSense SDK 2.0 is missing`  
To solve, update the file `realsense-ros/realsense_camera/CMakeLists.txt`,line: 43 to `find_package(realsense2 2.36.0)`
i.e. downgrade the required version of `realsense2` to `2.36.0`


## Set up ROS environment
Source your ROS environment one more time by running this command:

```sh
source ~/.bashrc
```


## Test installation

If you completed the preceding tasks successfully, you should be able to run this ROS launch command and see the Gazebo simulator and RViz simulator open with the following display:
```sh
roslaunch parc-robot task1.launch
```
![Gazebo Simulator window](assets/gazebo.png)
Gazebo Simulator window


![RViz window](assets/rviz.png)
RViz window


If you run the following command in a new terminal,
```
rqt_graph
```
You will see a screen like this:

![RQT Graph](assets/rosgraph.png)

You need to `publish`/write to the `topic` `/cmd_vel` to move the robot.
The following guide will help you control the robot using keyboard. Once you have tested that, you can follow the [understanding-ros](../getting-started-with-ros) guide to write a python program to control the robot.

## Controlling the robot using keyboard
Run the following command in a new terminal
```sh
source ~/catkin_ws/devel/setup.bash
roslaunch parc-robot teleop.launch
```

Now keeping the second terminal on top (teleop.launch) press `i` to move the robot forward, you can see the robot moving in "RViz" and "Gazebo" windows.
you can use the keys shown below to move the robot and `k` key to stop the movement.
```sh
Moving around:
   u    i    o
   j    k    l
   m    ,    .
```

## Developing inside the container with VSCode

1. Install the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension in VSCode.

2. Click on the green icon in the bottom left corner of the VSCode window and select `Open Folder in Container...`.

3. Select the `catkin_ws` folder.

4. VSCode will now open the `catkin_ws` folder inside the container.

5. You can now use VSCode to edit files in the `catkin_ws` folder.

Alternatively, since we have already created a volume for the `catkin_ws` folder, you can also use your favorite editor to edit files in the `catkin_ws` folder on your host machine. The changes will be reflected inside the container. The advantage of using VSCode in the container is that you can use the integrated terminal to run commands inside the container.
