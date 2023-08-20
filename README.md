# IRP5-Search-and-Rescue-Robot

Search and Rescue (SAR) refers to the vital process of locating and aiding individuals in perilous situations such as natural disasters. After such incidents, SAR teams aim to swiftly locate and assist survivors who are often in hazardous conditions. This process often possesses a certain amount of risk, however by leveraging robotics, SAR teams can navigate difficult-to-reach areas and save survivors more effectively without risking human lives.

In this project, the Multi-agent System for non-Holonomic Racing (MuSHR) platform will be modified to be used as a SAR robot. It will explore and document the use of the You Only Look Once (YOLO) algorithm for survivor detection as well as the implementation of localisation and planning algorithms using the Robot Operating System (ROS). The system was developed, tested, and tuned to improve performance. A final test showed that the survivor detection and navigation systems were working in tandem with one another. The systems developed in this project can be adjusted and applied to other robotic platforms.

This repository consists of the two ROS packages that were developed and used:
1. mushr_2dnav
   Package that implements the ROS Navigation Stack on the MuSHR platform
2. darknet_ros
   Package used to run YOLO to detect people. Also includes ROS node used to capture images from the camera footage whenever a person is detected.

References:
- https://github.com/prl-mushr
- https://github.com/leggedrobotics/darknet_ros
- https://github.com/Soft-illusion/Robotics_PicoDegree
- https://gist.github.com/hdh7485/f87b67b237ef57e46fe77962e343c28b
