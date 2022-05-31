# Disclaimer
This is a fork of [`dustynv's ROS2 nodes for NVIDIA Jetbot`](https://github.com/dusty-nv/jetbot_ros).
It was created for a study paper and will not be maintained further.
The repository was only created for evaluation purposes.
For a maintained version go to said repository.

I just added an API-Handler Node with a REST-Endpoint using Flask to get Pictures and Control the Motors from my Desktop.
Additionally I added a launch file which launches the needed nodes.
To reduce build times I also removed the Gazebo-Installation in the Dockerfile as it wasn't necessary for my paper.
That's it.
