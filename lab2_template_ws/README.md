# Lab 2 Template
This lab will introduce you to Automatic Emergency Braking (AEB), which is a node that will automatically stop the car when it is about to collide with an obstacle.

## Lab Objectives
- Learn how to program your first ROS node
- Learn basic operation of the AEB
- Learn how to create and use ROS 2 parameters
- Learn how to read LiDAR and odometry data and publish drive commands
- Learn how ROS 2 nodes communicate using publisher and subscriber

## Steps
### Setup your workspace
1. Navigate to where you want to keep the lab workspace, preferably home.
~~~
cd
~~~

2. Clone this branch from GitHub using the following command:
~~~
git clone -b driver_template https://github.com/Roboracer-Purdue/roboracer-purdue driver_template
~~~

3. Navigate into this lab's template
~~~
cd driver_template/lab2_template_ws
~~~
4. Type the following command to open the folder in VSCode (Get it installed if you are missing this part!)
~~~
code .
~~~

### Accessing the Source Code
1. In VS Code, open src/emergency_braking/emergency_braking/eb_template.py
2. This file is your source code, you will make changes to the emergency braking node here.

### Declare New Parameters
A ROS 2 node can be run with specific parameter values, which can be used as constants/configurations in the node. This step tells you how to declare ROS 2 parameters in our template.
1. Create a parameter named **TTC_threshold** and give it a value of about 0.2-0.5 by defining a new tuple in `declare_all_params()`function.
2. Create a parameter named **eb_topic** and give it a value of `'/eb'` by defining a new tuple in `declare_all_params()`function.
3. Load those parameter values into the node variables in the `load_params()` function.

### Complete the Control Loop
The Control Loop section (`control_loop()`) of the Emergency Braking Node given to you is missing several lines of code. (Highlighted as TODO), make sure to complete those line according to the instruction given in the comments. (Ask for help if you are stuck)

### Build Your Code
1. Make sure you saved your source code
2. Build the workspace after editing your code by navigating to the workspace folder and run the build command

~~~
cd lab2_template_ws
colcon build
~~~

### Test Your Node
1. Boot up a separate terminal, launch f1tenth_gym on it. This should be located in the testing_workspace you installed prior.
~~~
cd roboracer-purdue/roboracer_ws/
source install/local_setup.bash

ros2 launch f1tenth_gym_ros gym_bridge_launch.py
~~~
2. Boot up another separate terminal, run key_drive.py
~~~
cd roboracer-purdue/roboracer_ws/

python3 scripts/key_drive.py
~~~


3. Back to our original terminal, source your lab 2 workspace so ROS can see your package containing the AEB node. **You only need to do this once** for each terminal session.
~~~
source install/local_setup.bash
~~~

4. Run `emergency_braking` node from your `emergency_braking` package with parameter value of 0.3 for **TTC_threshold**
~~~
ros2 run emergency_braking emergency_braking --ros-args -p TTC_threshold:=0.3
~~~
Or simply run without parameters, it will use the default value you put in your `declare_all_params()`function.
~~~
ros2 run emergency_braking emergency_braking
~~~

5. Use your key_drive terminal to drive into a wall, your emergency braking node should be firing when it is about to hit the wall

6. Use '2d pos estimate' tool to teleport your car back if it got stuck.

7. To test it again, stop the brake node, teleport the car, and run the node again.

### Inspect ROS 2 Topics

Before moving on to the next step, take a moment to inspect how your ROS 2 nodes are communicating.

Open a **fourth terminal**. This terminal will be used to inspect ROS 2 topics while the simulator, `key_drive.py`, and your emergency braking node are still running.

1. Source the ROS 2 workspace so this terminal can access the running ROS graph.

```bash
cd roboracer-purdue/roboracer_ws/
source install/local_setup.bash
```

2. List the currently available topics.

```bash
ros2 topic list
```

You should see topics such as:

```text
/scan
/drive
/ego_racecar/odom
```

3. Inspect the `/scan` topic.

```bash
ros2 topic info /scan
```

This command shows the message type used by the topic and how many publishers and subscribers are currently connected.

To view the actual LiDAR messages being published, run:

```bash
ros2 topic echo /scan
```

Press `Ctrl+C` when you are finished viewing the messages.

4. Inspect the `/drive` topic.

```bash
ros2 topic info /drive
```

At this point, you may notice that `/drive` has more than one publisher. Both `key_drive.py` and the emergency braking node can publish drive commands to the same topic.

You can also inspect the commands being sent to the vehicle:

```bash
ros2 topic echo /drive
```

Try pressing `w`, `s`, `a`, and `d` in the `key_drive.py` terminal and observe how the values published to `/drive` change.

You have now seen how ROS 2 topics can be inspected from the command line. Keep this fourth terminal open—we will use it again later to inspect the `/eb` topic that you create.

### Dealing with key_drive.py
At this point, you may have noticed that the emergency node publishes 0 speed and 0 steering to `/drive` to stop the car. However, your car is still drifting slowly forward, eventually colliding with the wall.

This is because both the brake node and the key_drive node are publishing different speeds to `/drive` simultaneously. And they are fighting to drive the car in a different way.

To fix this, the emergency node will publish a message to the `/eb` topic, and the KeyDrive node will listen to that `/eb` topic.

For this lab, `/eb` carries an emergency speed command. When the brake is triggered, AEB publishes `0.0`, causing key_drive to replace its current commanded speed with zero.

1. In your AEB Node, under `__init__()`, create a new publisher called `eb_pub` to publish a `Float32` value to the `/eb` topic. Use the existing drive_pub as an example.

2. Back to the control loop. Under the if statement, when the car is braking, publish a 0.0 to `/eb` topic using these lines.
~~~
    msg = Float32()
    msg.data = 0.0

    self.eb_pub.publish(msg)
~~~

3. Stop your currently running emergency braking node with `Ctrl+C`. Build your changes, source the workspace again, and restart the node.
~~~
colcon build
source install/local_setup.bash
ros2 run emergency_braking emergency_braking
~~~

Return to the fourth terminal you opened earlier and check whether your new `/eb` topic exists:

```bash
ros2 topic list
```

Inspect the topic:

```bash
ros2 topic info /eb
```

How many publishers and subscribers does `/eb` currently have?

Now inspect the messages being published:

```bash
ros2 topic echo /eb
```

Drive the car toward a wall. When emergency braking activates, you should see:

```text
data: 0.0
```

Press `Ctrl+C` to stop viewing the topic.

At this point, your emergency braking node can **publish** an emergency command, but nothing is listening to it yet. In the next steps, you will modify `key_drive.py` to **subscribe** to `/eb`.

4. Edit your key_drive.py (located in scripts/key_drive.py). 

5. Put in this line to use `Float32` type of message
~~~
from std_msgs.msg import Float32 
~~~

6. Add a subscriber to `/eb` topic under `__init__()`, use the subscriber in eb_template.py as example. Use this callback function name for now: `eb_callback`.

7. Add this callback function to **KeyDrive** node. We will make it so that when it receives a float from `/eb`, it will set current speed to that value.

~~~
def eb_callback(self, msg):
    self.speed = msg.data
~~~

8. Add this line to main, in the while loop to make sure the node process callbacks
~~~
rclpy.spin_once(node, timeout_sec=0.0)
~~~

9. Run your modified `key_drive.py` and emergency braking node again.

Return to your fourth terminal and run:

```bash
ros2 topic info /eb
```

Compare the result with what you saw earlier. `/eb` should now have both a **publisher** and a **subscriber**.

Test the complete system by driving toward a wall again. When the AEB node detects an imminent collision:

1. The AEB node publishes `0.0` to `/eb`.
2. `key_drive.py` receives the message through its `/eb` subscriber.
3. `eb_callback()` sets the commanded speed to `0.0`.
4. `key_drive.py` begins publishing the zero-speed command to `/drive`.

You can use:

```bash
ros2 topic echo /eb
```

or:

```bash
ros2 topic echo /drive
```

to observe the messages while testing.