launch gazebo world
launch perceptions (gotham_perceptions.launch.py) (color, qr, text)

at the maze:
run mission_manager (start Joker Mission)
run hunger_games

once it detect joker zone:
1)stop autonomous
2) send Joker's location

Manually do mission then scan QR "DONE"
1) sends message to mission_control abt mission done


here run object_node and close perception launch first


Then send_goal for autonomous again. (Start Penguin Mission)

ros2 action send_goal /patrol_mission action_tutorials_interfaces/action/PatrolMission "{}"


Moves autonomously till detects Penguin

Penguin_zone then stops again

off object_node, launch perception

do mission

then scan QR "DONE"

then send_goal again

then detect riddler text and stop

then do mission

then detect QR "DONE"

then send_goal

