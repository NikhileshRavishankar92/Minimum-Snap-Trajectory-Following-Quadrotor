# Minimum-Snap-Trajectory-Following-Quadrotor
Compute the minimum snap trajectory for a given set of waypoints and design a control algorithm for a quadrotor to follow that trajectory

![Alt text](https://github.com/NikhileshRavishankar92/Minimum-Snap-Trajectory-Following-Quadrotor/blob/master/Min_snap_traj.jpg)

![Alt text](https://github.com/NikhileshRavishankar92/Minimum-Snap-Trajectory-Following-Quadrotor/blob/master/Helix_traj.jpg)

![Alt text](https://github.com/NikhileshRavishankar92/Minimum-Snap-Trajectory-Following-Quadrotor/blob/master/Line_traj.jpg)

# Setup Instructions

Note: The quadrotor simulator was designed as a part of the Aerial Robotics MOOC on Coursera.

Copy all the files into your MATLAB directory. Make sure to add them to your MATLAB path. The runsim.m file runs the quadrotor simulation with the designed PD controller (in the controller.m file) for the specified trajectory (in the traj_generator.m file). Line , Helix and a minimum snap trajectory through a given set of waypoints are among the three choices than you can choose to set your "trajhandle" in runsim.m. 

Note: Each trajectory requires it's own set of PD control parameters. There is some amount of knowledge to be gained in tuning your controller in order to achieve good command following (Ziegler Nichols heurestic rules for tuning are pretty useful). But after you get close to your desired commands, further tuning is more a process of trial and error. This can be automated by a handly algorithm called the twiddle alorithm (for more information on this, please check the AI for robotics course at Udacity, I'll try and upload an implementation of the twiddle algorithm for the 2D case soon).
