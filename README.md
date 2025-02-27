# Repo for Exercise 3 cmput 412

First run `dts devel build -H csc22907.local -f` to build the executables on the duckie bot

## Part 1
Distortion:
- To view the undistorted image run ` dts devel run -H csc22907.local -L color-detection`
- Then in another terminal run `dts start_gui_tools csc22907` and then `rqt_image_view` Then you can see the undistorted and blurred image under the topic `/camera_undistorted/image/compressed`

Color detection:
- To view the color detection run ` dts devel run -H csc22907.local -L color-detection`
- Then in another terminal run `dts start_gui_tools csc22907` and then `rqt_image_view` Then you can see the color detection and contour under the topic `/lane_detection/image/compressed`

LED controller
- Led can be controlled by importing LEDBlinker from `led_service` file. It has 2 functions set_let_color which takes a color and turns all the color to given color and another function set_led_color2 which takes a color and an array of 5 to indicate which led to turn 0 meaning off and 1 in the array meaning change color

Autonomous navigation:
- Autonomous navigation is controlled by `auto_nav_func` file. It has 3 functions:
    - move_straight(distance(in metre)
    - move_turn_left()
    - move_turn_right()

Lane Based behaviour
- Run the command ` dts devel run -H csc22907.local -L lane-based`
- If it detects red, move up to the red and move straight again
- detects blue: move up to blue and turn left
- detects green: move up to green and turn right
##### The program is programmed to stop after executing one lane-based behavior. To see behavior in different color run the program in front of different color 
