## Packages to control baxter


### Execute simulation

start baxter simulation


spawn coke model

    rosrun gazebo_ros spawn_model -file /home/rick/.gazebo/models/coke_can2/model.sdf -sdf -x 0.5 -y 0.5 -z 1 -model coke_can2


move coke model

    rostopic pub /gazebo/set_model_state gazebo_msgs/ModelState "model_name: 'coke_can2'
    pose:
    position:
        x: 0.5
        y: 0.5
        z: 0.5" 

start inverse kinematic script

    rosrun baxter_control_sofar ik_service_client2_topic.py -l left


send end effector goal

    rostopic pub positn_sub geometry_msgs/Point "x: 0.5
    y: 0.4
    z: -0.1" 
