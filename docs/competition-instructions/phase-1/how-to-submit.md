# How to Submit

Teams are expected to develop their solutions (ROS packages) in a 'solutions' folder inside the `~/catkin_ws/src` directory. You may have one or more ROS packages in this folder for all your tasks. 

See figure below of expected directory structure:

```
~/catkin_ws/src
├── CMakeLists.txt
├── parc-engineers-league
│   ├── parc-robot
│   │   ├── .
│   │   ├── .
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── .
│   └── .
└── <YOUR_SOLUTION_FOLDER>        # Zip this folder and submit
    ├── <your_ros_package1>
    │   ├── .
    │   ├── .
    │   ├── CMakeLists.txt
    │   └── package.xml
    ├── <your_ros_package2>
    │   ├── .
    │   ├── .
    │   ├── CMakeLists.txt
    │   └── package.xml
    ├── .
    ├── .
    ├── .
    ├── .
    └── README.md                   # Required
```

1. Prepare a README.md file following this format and store in solution folder (see [example](https://github.com/PARC-Robotics/PARC-Engineers-League/blob/develop/resources/sample-submission-readme.md)):
    * Introduction Section: Briefly describe your approach
    * Dependencies: List all the packages installed and used in your solution
    * Task 1 - 3 description and run command(s)
    * Challenges faced

2. Include all the packages (dependencies) used in your solution in your package's "package.xml" file ([see guide](http://wiki.ros.org/rosdep/Tutorials/How%20to%20add%20a%20system%20dependency))
3. Zip your solution folder and upload on the [solution submission form](https://google.com) [TBA]
