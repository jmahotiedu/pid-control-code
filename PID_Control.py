import mujoco
import numpy as np 

class JointSpacePDController: 
    def __init__(self, model, joint_ids=None, actuator_ids=None): 
        self.model = model 

        # Assume 14 joints / actuators unless specified 
        self.joint_ids = joint_ids if joint_ids is not None else list(range(14)) 
        self.actuator_ids = actuator_ids if actuator_ids is not None else list(range(14)) 

        assert len(self.joint_ids) == 14 
        assert len(self.actuator_ids) == 14 

        # PD gains (conservative defaults) 
        # ERROR: The arms are funtioning as if they are not mirrored, setting the gains both at 100
        # cause them to try and be in the same exact position, the right arm spasms because it is trying 
        # to function like the left arm
        self.Kp = np.ones(14) * 100
        print(self.Kp)
        self.Kd = np.ones(14) * 100
        print(self.Kd)

        # Actuator limits 
        self.ctrl_min = model.actuator_ctrlrange[self.actuator_ids, 0] 
        self.ctrl_max = model.actuator_ctrlrange[self.actuator_ids, 1] 

 
 

    def compute(self, data, q_des): 

        """ 

        Compute joint torques using PD control + gravity compensation. 

        Parameters 

        ---------- 

        data : mujoco.MjData 

        q_des : np.ndarray (14,) 

            Desired joint positions 

        Returns 

        ------- 

        tau : np.ndarray (14,) 

            Joint torques 

        """ 

 
 

       # Current joint positions and velocities
        q = data.qpos[self.joint_ids]
        qd = data.qvel[self.joint_ids]

        # Position and velocity errors
        pos_error = q_des - q
        vel_error = -qd

        # PD term
        tau = self.Kp * pos_error + self.Kd * vel_error

        # Gravity + Coriolis + centrifugal compensation
        rne_result = np.zeros(self.model.nv)
        mujoco.mj_rne(self.model, data, 0, rne_result)

        tau += data.qfrc_bias[self.joint_ids]

        # Clip torques
        tau = np.clip(tau, self.ctrl_min, self.ctrl_max)

        return tau
 