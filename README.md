# "Easy" Docker Containers

This repository contains a script to help make it easier creating dockerized environments that come preloaded with ROS and some other niceties so that we get to the fun part ASAP.

Supports versions of ROS:
- noetic

Also, the images aren't stable yet so if you're going to be running hard to setup environments, with things like Cuda/Torch/etc., come talk to me first.

This is all aimed at Linux. Probably Ubuntu, since that's what I tested[^*].

## How to use this

Clone the repository to your computer:
```
git clone https://github.com/hannwern/RPL-ARI-docker.git
```

Then you can add the environment script to your `.bashrc` so that the commands are always available as soon as open a terminal by doing:
```
cd RPL-ARI-docker
echo "source $(pwd)/environment" >> ~/.bashrc
```

If you have an Nvidia GPU, you can change the `Dockerfile` and `environment` file to use these. Important lines of code: Dockerfile.gallium lines 36-41.



Close & open a new terminal. The main idea now is that you can create a container that will mount one of the folders in your computer to the `src` folder of a catkin workspace in the container, and another of your folders to a folder I call `bridge` so that you can move files in and out of the container easily.

First create the src and bridge folders
```
> mkdir -p ~/path/to/code/src ~/path/to/code/bridge
```


If you want to use the gallium docker file, make sure to log in to gitlab, and that you have access to the Ari docker files. First log into gitlab and create a personal access token, then:
```
> docker login registry.gitlab.com
```
when prompted, enter your gitlab user name (email) and use the personal access token you just created as the password.


Next, create the container (here the ros distro gallium is used but just change it for the other ones (noetic) if that's what you need). 
```
> create_container gallium container_name ~/path/to/code/src ~/path/to/code/bridge
```

You only have to run `create_container` once. From then on, to start it up again just run `start_container`:
```
start_container container_name
open_terminal container_name
```

Now you'll be in a terminal inside your container. If you need more, just open more terminals and run `open_terminal container_name`. 

Anything you do to the `catkin_ws/src` and `bridge` folders in the container is also always available to you in your own folders `~/path/to/code/src` and `~/path/to/code/bridge`, and vice versa. Therfore you can run your editor of choice outside the Docker and just use the terminals to compile things.

**SUPER MEGA WARNING**: The connection between the `src` folders and the `bridge` folders in your computer and in your container is total. **Any** change you make, regardless of if it's on the container or your computer, happens on both. Something you delete is gone from both sides.


**WARNING**: Only the folders `catkin_ws/src` and `bridge` are safe, since mounted outside the container. Anything else that is not in these folders is lost by deleting the container, so be careful when doing this that you've saved everything you don't want to lose in one of these folders.


To stop a container, run:
```
close_container container-name
```

To delete a container, run:
```
delete_container container-name
```



## Configure your hosts file

To make life easier for you, add the following to the `/etc/hosts` file in *YOUR* computer, not inside any of the containers:
```
192.168.0.42  ari-30c
```

You have to edit the file with sudo, so open the file with something like:
```
gksudo gedit /etc/hosts
```

## ROS Master or Slave

Once inside the container terminal, you can easily change between being a ROS Master and a ROS Slave. To make the terminal you're in behave as if the ROS Master is running in your computer, run:
```
> ros_master
```

If you want the terminal you're in to behave as a slave to some ROS Master, run:
```
> ros_slave MASTER_IP
```

Masters will likely be the ARI control computer, which has a hostname if you have configured your hosts file (look above). This means you can do it like this, :
```
> ros_slave ari-30c # For the ARI team
```

Whatever ROS project you run now in this "slave" terminal will run on Ari.


## What's in the container

ROS and little else. I made sure there's also a text editor (nano/vim) and git. Feel free to add other thongs you might need by editing the dockerfile.

 **EDIT**: I removed the need for a password for sudo commands.
If you run a sudo command i made sure the password is `passwd`. If you find yourself in the root user for some reason, the password is `root`.


## Troubleshooting
1. Rviz might give a dbus error. If it does, run the following command in the terminal you're running Rviz in:
```
DBUS_SESSION_BUS_ADDRESS=/dev/null rosrun rviz rviz -d `rospack find ari_2dnav`/config/rviz/navigation.rviz
```

If this fails, try creating container without Nvidia. Then try step 1 again
