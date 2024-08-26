# "Easy" Docker Containers

This repository contains a script to help make it easier creating dockerized environments that come preloaded with ROS and some other niceties so that we get to the fun part ASAP.

Supports versions of ROS:
- noetic

Also, the images aren't stable yet so if you're going to be running hard to setup environments, with things like Cuda/Torch/etc., come talk to me first.

This is all aimed at Linux. Probably Ubuntu, since that's what I tested[^*].

[^*]: I haven't been able to test that graphics forwarding, like Rviz works yet.

## How to use this

Clone the repository:
```
git clone https://github.com/hannwern/RPL-ARI-docker.git
```

Then you can add the environment script to your `.bashrc` so that the commands are always available as soon as open a terminal by doing:
```
cd RPL-ARI-docker
echo "source $(pwd)/environment" >> ~/.bashrc
```

Close & open a new terminal. The main idea now is that you can create one container for each version of ROS. Each containers will mount one of your folders to the `src` folder of a catkin workspace and another of your folders to a folder I call `bridge` so that you can move files in and out of the container easily.

I suggest you try this out by running (I'm using gallium here, but just change it for the other ones if that's what you need):
```
cd ~
mkdir -p noetic-container/src noetic-container/bridge
make_container noetic ~/noetic-container/src ~/noetic-container/bridge
start_container noetic
open_terminal noetic
```

Now you'll be in a terminal inside your noetic container. If you need more terminals, just open more terminals and run `open_terminal noetic`. Anything you do to the `catkin_ws/src` and `bridge` folders in the containers is also always available to you in your own folders `noetic-container/src` and `noetic-container/bridge`, so you can run your editor of choice outside the Docker and just use the terminals to compile things.

If you want to connect your editor to the container for something like debugging, I can vouch for VSCode for Python. Guide for setup is [here](https://code.visualstudio.com/docs/devcontainers/tutorial).

To stop a container, run:
```
close_container noetic
```
You only have to run `make_container` the first time (minus if your container gets ruined, more on that later). From then on, to start it up again just run `start_container`:
```
start_container noetic
open_terminal noetic
```

**SUPER MEGA WARNING**: The connection between the `src` folders and the `bridge` folders in your computer and in your container is total. **Any** change you make, regardless of it's on the container or your computer, happens on both. Something you delete is gone from both sides.

### CUDA/Torch/Python versions/etc. went wrong
If you've irreparably damaged your container by installing/uninstalling things you don't really want, you can delete it and create a new one:
```
delete_container noetic
make_container noetic ~/noetic-container/src ~/noetic-container/bridge
```

**WARNING**: Only the folders `catkin_ws/src` and `bridge` are safe, since mounted outside the container. Anything else that is not in these folders is lost by deleting the container, so be careful when doing this that you've saved everything you don't want to lose in one of these folders.

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

ROS and little else. I made sure there's also a text editor (nano/vim) and git. Things you should install yourself via `apt`. If you run a sudo command, the password is `passwd`. If you find yourself in the root user for some reason, the password is `root`.

## What's missing?

Graphics only work with X11 and if you have Wayland the comands break. If you get an error with something related to `/tmp/.X11`, talk to me.