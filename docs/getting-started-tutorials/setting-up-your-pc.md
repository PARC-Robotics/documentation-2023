# Setting up your PC

This guide helps you setup your computer for running the competition environment locally and developing the code.

You can use local Computer/Laptop or Virtual Machine inside your computer or any cloud platform like [Google GCP](https://cloud.google.com/free){target=_blank}, [Amazon AWS](https://aws.amazon.com/free/){target=_blank}, [Microsoft Azure](https://azure.microsoft.com/en-us/free/){target=_blank}, [Digital Ocean](https://try.digitalocean.com/freetrialoffer/){target=_blank}, etc (All Cloud providers have some free trial plan which you can make use of).
## System requirements

The competition setup needs to be run on [Ubuntu](https://ubuntu.com/download){target=_blank}, a flavor of [Linux](https://en.wikipedia.org/wiki/Linux){target=_blank}. You will need a computer that has:
    
- A dedicated [GPU](https://en.wikipedia.org/wiki/Graphics_processing_unit){target=_blank},
    - Nvidia cards tend to work well in Ubuntu
- A CPU that is at least an Intel i5, or equivalent,
- At least 4GB of free disk space,
- At least 8GB of RAM,
- [Ubuntu Focal Fossa](https://releases.ubuntu.com/focal/){target=_blank} installed.

## Operating System
If not already installed, Install **[Ubuntu Focal (20.04)](https://releases.ubuntu.com/focal/){target=_blank}** on the system by following [this guide](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview){target=_blank}.

!!! note
    It is highly recommended to install [Focal (20.04)](https://releases.ubuntu.com/focal/){target=_blank} version of Ubuntu due to [ROS (Noetic)](http://wiki.ros.org/noetic){target=_blank} dependency.

## Installing ROS
You need to install ROS Noetic by following [this guide](http://wiki.ros.org/noetic/Installation/Ubuntu){target=_blank} and install `ros-noetic-desktop-full` in the step `1.4` of the guide.

If you prefer video instructions, you can follow these videos:

- [Installing Ubuntu 20.04 on VirtualBox](https://www.youtube.com/watch?v=x5MhydijWmc){target=_blank}
- [Installing ROS Noetic on Ubuntu 20.04](https://www.youtube.com/watch?v=ZEfh7NxLMxA){target=_blank}

## Next Steps

To set up your workspace, there are two available options. You can either use Docker or manually configure your workspace. If you prefer to configure your workspace manually, follow the instructions provided [here](../getting-started-tutorials/setting-up-your-workspace.md). Alternatively, if you would like to set up your workspace using Docker, follow the instructions provided [here](../getting-started-tutorials/setting-up-your-workspace-using-docker.md).
