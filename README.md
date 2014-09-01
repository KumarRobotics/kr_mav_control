quadrotor
=========

ROS packages for quadrotor control

Note: There are a couple packages that require submodules.

### Submodules

###### Package (submodule):

quadrotor\_simulator (odeint)

quad\_serial\_comm (asio\_serial\_device)


Initialize the submodules by running the following in the repository directory:

    $ git submodule init
    $ git submodule update

Note: In ROS Indigo on Ubuntu 14.04, you need to make symlinks for boost for quad_serial_comm.
For example:

    sudo ln -s /usr/lib/x86_64-linux-gnu/libboost_system.so /usr/lib/
    sudo ln -s /usr/lib/x86_64-linux-gnu/libboost_thread.so /usr/lib/
