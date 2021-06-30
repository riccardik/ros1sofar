# Packages to control baxter's left end effector position


## Steps to Execute simulation

- execute the simulation script: `./baxter.sh`
- start baxter simulation, when it has spawned correctly launch the second launch file in a new terminal window: (this will include all the steps below)

    `roslaunch baxter_control_sofar baxter_world.launch `
    `roslaunch baxter_control_sofar spawn.launch `

- start each node separately

    start baxter simulation

        roslaunch baxter_control_sofar baxter_world.launch

    spawn coke model

        rosrun gazebo_ros spawn_model -file /home/rick/.gazebo/models/coke_can2/model.sdf -sdf -x 0.5 -y 0.5 -z 1 -model coke_can2

    start inverse kinematic script

        rosrun baxter_control_sofar ik_service_client2_topic.py -l left


## Interact with the simulation

### Coke model
move coke model by publishing on the gazebo topic:

    rostopic pub /gazebo/set_model_state gazebo_msgs/ModelState "model_name: 'coke_can2'
    pose:
    position:
        x: 0.5
        y: 0.5
        z: 0.5" 

in alternative open a new bash script and run this command:

    rosrun baxter_control_sofar move_coke_topic.py

publish point on the topic  `/coke_can_coords`
    rostopic pub /coke_can_cods geometry_msgs/Point "x: 0.5
    y: 0.4
    z: 0.8" 


### send a goal to the end effector

rostopic pub position_sub geometry_msgs/Point "x: 0.5
y: 0.4
z: -0.1" 
