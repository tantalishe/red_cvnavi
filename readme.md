How to run line finder?
==================

NODE_NAME=cv_linetochka

1) Calibrate camera or check if the file exists.
```
$ cd ~/.ros/camera_info && ls
	logitech.yaml
```

2) Launching usb_cam

	    TODO: rect colored image

	For local computer:

		$ roslaunch red_cvision usb_cam_local_0.launch

	where "0" is a number of /dev/video*

	For computer of youbot:

		$ roslaunch red_cvision usb_cam_youbot_0.launch

3) Launching node

        $ rosrun red_cvnavi cvnavi.py

    for default alpha = 18.3, normal = 0.52

    or

	    $ rosrun red_cvnavi cvnavi.py -a ANGLE -n NORMAL

    -a -- alpha angle -- camera rotate [deg];
    -n -- normal length from camera to floor [m];

4) [for start]

    Turn on Line Finder:

	    $ rosservice call /camera_floor "switch: true"

    Turn off Line Finder:

        $ rosservice call /camera_floor "switch: false"

5) [just for enjoy] See image

	    $ rosrun image_view image_view image:=/see_tape

6) Glooping on useful information:

        $ rostopic echo /camera_tape

Have fun!
