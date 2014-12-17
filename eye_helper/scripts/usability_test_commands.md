### one terminal per robot
```
roslaunch eye_helper multicamera_eye_helper.launch host:=192.168.17.206 robotname:=usercam
roslaunch eye_helper multicamera_eye_helper.launch host:=192.168.17.202 robotname:=sidecam
roslaunch eye_helper multicamera_eye_helper.launch host:=192.168.17.209 robotname:=frontcam
```

IP addresses:
look on the raspberry pi

robotnames:
usercam
sidecam
frontcam

eye_helper.py should be subscribed to the proper topic (usercam)

```
roscore # restart roscore right before you start doing things
rosrun eye_helper eye_helper.py
rosbag record /usercam/camera/image_raw/compressed /sidecam/camera/image_raw/compressed /frontcam/camera/image_raw/compressed 
```

alternatively, rosbag only the usercam
```
rosbag record /usercam/camera/image_raw/compressed  
```

TODO: put the rosbags somewhere