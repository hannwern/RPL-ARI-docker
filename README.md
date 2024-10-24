# "Easy" Docker Containers

This repository contains a script to help make it easier creating dockerized environments that come preloaded with ROS and some other niceties so that we get to the fun part ASAP.

Supports versions of ROS:
- noetic

Also, the images aren't stable yet so if you're going to be running hard to setup environments, with things like Cuda/Torch/etc., come talk to me first.

This is all aimed at Linux. Probably Ubuntu, since that's what I tested[^*].

[^*]: I haven't been able to test that graphics forwarding, like Rviz works yet.

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

If you have an Nvidia GPU, you can change the `Dockerfile` and `environment` file to use these. In the docker file line 8 was commented out, and line 45-47 was added. In the environment file the “docker run …” for Nvidia is commented out. 


Log in to gitlab to access the Ari docker files. First log into gitlab and create a personal access token, then:
```
> docker login registry.gitlab.com
```
when prompted, enter your gitlab user name (email) and use the personal access token you just created as the password.


Close & open a new terminal. The main idea now is that you can create a container that will mount one of the folders in your computer to the `src` folder of a catkin workspace in the container, and another of your folders to a folder I call `bridge` so that you can move files in and out of the container easily.

First create the src and bridge folders
```
> mkdir -p ~/path/to/code/src ~/path/to/code/bridge
```


Create the container (here the ros distro gaallium is used but just change it for the other ones if that's what you need):
```
> make_ari_container gallium container_name ~/path/to/code/src ~/path/to/code/bridge
```


You only have to run `make_container` the first time (minus if your container gets ruined, more on that later). From then on, to start it up again just run `start_container`:
```
start_ari_container container_name
open_ari_terminal container_name
```

Now you'll be in a terminal inside your container. If you need more terminals, just open more terminals and run `open_terminal container_name`. 

Anything you do to the `catkin_ws/src` and `bridge` folders in the container is also always available to you in your own folders `~/path/to/code/src` and `~/path/to/code/bridge`, and vice versa. Therfore you can run your editor of choice outside the Docker and just use the terminals to compile things.

**SUPER MEGA WARNING**: The connection between the `src` folders and the `bridge` folders in your computer and in your container is total. **Any** change you make, regardless of it's on the container or your computer, happens on both. Something you delete is gone from both sides.


**WARNING**: Only the folders `catkin_ws/src` and `bridge` are safe, since mounted outside the container. Anything else that is not in these folders is lost by deleting the container, so be careful when doing this that you've saved everything you don't want to lose in one of these folders.


To stop a container, run:
```
close_ari_container container-name
```

To delete a container, run:
```
delete_ari_container container-name
```



## Configure your hosts file

To make life easier for you, add the following to the `/etc/hosts` file in *YOUR* computer, not inside any of the containers:
```
192.168.100.177 Gimli
192.168.100.179 aragorn
192.168.100.142 arrakis
192.168.100.42  ari-30c
```

You have to edit the file with sudo, so open the file with something like:
```
gksudo gedit /etc/hosts
```

## ROS Master or Slave

Once inside the container, you can easily change between being a ROS Master and a ROS Slave. To make the terminal you're in behave as if the ROS Master is running in your computer (might be in another container, doesn't necessarily have to be the one you are running this in), run:
```
(noetic) hackathon@arrakis$ ros_master
```

If you want the terminal you're in to behave as a slave to some ROS Master, run:
```
(noetic) hackathon@arrakis$ ros_slave MASTER_IP
```

For the hackathon the masters will likely either be the ARI or the YuMi's control computer, which both have hostnames. This means you can do it like this, if you have configured your hosts file (look above):
```
(noetic) hackathon@arrakis$ ros_slave ari-30c # For the ARI team
(noetic) hackathon@arrakis$ ros_slave Gimli # For the YuMi team
```

## What's in the container

ROS and little else. I made sure there's also a text editor (nano/vim) and git. Feel free to add other thongs you might need.

If you run a sudo command, the password is `passwd`. If you find yourself in the root user for some reason, the password is `root`. EDIT: I removed the need for a password for sudo commands.




## Troubleshooting

1. Rviz might give a dbus error. If it does, run the following command in the terminal you're running Rviz in:
```
DBUS_SESSION_BUS_ADDRESS=/dev/null rosrun rviz rviz -d `rospack find ari_2dnav`/config/rviz/navigation.rviz
```

2. Try creating container without Nvidia. Then try step 1
