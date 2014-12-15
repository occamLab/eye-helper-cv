### one terminal per robot
```
$ roscore
multicamera_eye_helper.launch host:=<IP address> robotname:=usercam
$ roscore
multicamera_eye_helper.launch host:=<IP address> robotname:=sidecam
$ roscore
multicamera_eye_helper.launch host:=<IP address> robotname:=frontcam
```

IP addresses:
look on the raspberry pi

robotnames:
usercam
sidecam
frontcam

eye_helper.py should be subscribed to the proper topic


```
$ roscore # restart roscore right before you start doing things
$ rosrun eye_helper eye_helper.py
$ rosbag record /<usercam>/camera/image_raw/compressed /<sidecam>/camera/image_raw/compressed /<frontcam>/camera/image_raw/compsideed 
```