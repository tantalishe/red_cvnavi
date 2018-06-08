How to run line finder?
==================

NODE_NAME=cv_tape_dtc

1) Change camera topic for your own camera

2) Check TF frame names

3) Launching node

        $ rosrun red_cvnavi cvnavi.py

4) [for start]

    Turn on Line Finder:

	    $ rosservice call /camera_floor "switch: true"

    Turn off Line Finder:

        $ rosservice call /camera_floor "switch: false"

5) [just for enjoy] See image

	    $ rosrun image_view image_view image:=/see_tape

6) Glooping on useful information:

        $ rostopic echo /red_tape
    and
    	$ rostopic echo /yellow_tape

Have fun!
