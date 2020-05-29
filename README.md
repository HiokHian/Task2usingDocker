# Task2usingDocker
Running Task 2 from a Docker Image. Dockerfile was used to build the image.To replicate setting up of ROS nodes, refer to README.
Comments that provide further explanation are preceded by a '#'.


# Building docker image from dockerfile:

1) cd into the folder
2) docker build -t task2image .
(docker will find "Dockerfile" and start to build the image)



# Setting up the ROS nodes:

1) Open up 3 terminals
2) Plug in Arduino Board into USB port that has the name /dev/ttyACM0 (project currently assumes that the code in ToggleLEDArduinoSubscriberClient.ino has already been uploaded to the Arduino Uno)
3) Follow the following steps sequentially

Go to 1st terminal: (ros subscriber and client)
1) docker run -it --device=/dev/ttyACM0 task2image bash  #to include ACM0 port in the docker environment
2) rosrun rosserial_python serial_node.py /dev/ttyACM0

Next, go to 2nd terminal: (roscore)
1) docker ps #get the running container id
2) docker exec -it {running container ID} bash
3) source ./devel/setup.bash
4) roscore

Next, go to 3rd terminal: (ros publisher and server)
1) docker ps #get the running container id
2) docker exec -it {running container ID} bash
3) source ./devel/setup.bash
4) rosrun pypublish main.py

Lastly, go back to 1st terminal:
1) Ctrl+C #shut down the ros subscriber and client and start it up again
2) rosrun rosserial_python serial_node.py /dev/ttyACM0
