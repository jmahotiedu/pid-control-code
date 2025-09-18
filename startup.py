import mujoco 
import mujoco.viewer 
import numpy as np 
import time 
from PID_Control import JointSpacePDController  # or paste class above 


# ----------------------------------------------------- 
# Load model 
# ----------------------------------------------------- 

model = mujoco.MjModel.from_xml_path("aloha.xml") 
data = mujoco.MjData(model) 


# ----------------------------------------------------- 
# Create controller 
# ----------------------------------------------------- 

controller = JointSpacePDController(model) 


# ----------------------------------------------------- 
# Desired joint configuration 
# ----------------------------------------------------- 

q_des = data.qpos[:14].copy() 



# Small offset to visibly move one joint 
# This is moving the joints of each arm based on the value given, 
# the joints may be incorrect as all of the joints are moving when the program is run
q_des[0] += 3.14      # left arm joint 1 
q_des[6] += 3.14     # right arm joint 1 
print("q_des[0]: ", q_des[5])
print("joint range: ", model.jnt_range[8])

  
# ----------------------------------------------------- 
# Launch viewer 
# ----------------------------------------------------- 

with mujoco.viewer.launch_passive(model, data) as viewer: 
    while viewer.is_running(): 

        # Compute torques 
        tau = controller.compute(data, q_des) 

        # Apply torques 
        data.ctrl[:] = tau 

        # Step simulation 
        mujoco.mj_step(model, data) 

        # Sync viewer 
        viewer.sync() 

        # Run at real-time speed
        time.sleep(model.opt.timestep) 

 

 