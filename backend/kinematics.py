import numpy as np
from scipy.optimize import minimize
import math

class RobotArm:
    def __init__(self, dh_params):
        """
        dh_params: list of dicts, each containing:
        {'theta_offset': float, 'd': float, 'a': float, 'alpha': float}
        """
        self.dh_params = dh_params
        self.dof = len(dh_params)

    def dh_matrix(self, theta, d, a, alpha):
        """Computes the Denavit-Hartenberg transformation matrix."""
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,             np.sin(alpha),               np.cos(alpha),               d],
            [0,             0,                           0,                           1]
        ])

    def forward_kinematics(self, joint_angles):
        """
        Calculates FK.
        Returns a list of transformation matrices for each joint.
        The last matrix is the end-effector frame.
        """
        frames = [np.eye(4)] # Base frame
        T = np.eye(4)
        
        for i in range(self.dof):
            # theta is the variable joint angle plus any offset
            theta = joint_angles[i] + self.dh_params[i]['theta_offset']
            d = self.dh_params[i]['d']
            a = self.dh_params[i]['a']
            alpha = self.dh_params[i]['alpha']
            
            A_i = self.dh_matrix(theta, d, a, alpha)
            T = np.dot(T, A_i)
            frames.append(T.copy())
            
        return frames

    def inverse_kinematics_analytical(self, target_pos, current_angles=None):
        """
        Attempts exact analytical IK for specific 2DOF and 3DOF configurations.
        Returns a list of joint angles, or None if unreachable/not supported.
        """
        x, y, z = target_pos
        
        if self.dof == 2:
            # Assume planar robot in X-Y plane
            L1 = self.dh_params[0]['a']
            L2 = self.dh_params[1]['a']
            
            # Check reachability
            dist = np.sqrt(x**2 + y**2)
            if dist > L1 + L2 or dist < abs(L1 - L2):
                return None # Unreachable
                
            c2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
            c2 = np.clip(c2, -1.0, 1.0) # avoid precision issues
            
            q2 = math.acos(c2) # elbow down (or could be -q2 for elbow up)
            
            k1 = L1 + L2 * np.cos(q2)
            k2 = L2 * np.sin(q2)
            
            q1 = math.atan2(y, x) - math.atan2(k2, k1)
            
            return [q1, q2]
            
        elif self.dof == 3:
            # Spatial arm: Base rotation (around Z), then 2 planar joints
            L1 = self.dh_params[0]['d'] # Base height
            L2 = self.dh_params[1]['a']
            L3 = self.dh_params[2]['a']
            
            q1 = math.atan2(y, x)
            
            # Project to the arm plane
            r = np.sqrt(x**2 + y**2)
            z_prime = z - L1
            
            dist = np.sqrt(r**2 + z_prime**2)
            if dist > L2 + L3 or dist < abs(L2 - L3):
                return None
                
            c3 = (r**2 + z_prime**2 - L2**2 - L3**2) / (2 * L2 * L3)
            c3 = np.clip(c3, -1.0, 1.0)
            
            q3 = math.acos(c3)
            
            k1 = L2 + L3 * np.cos(q3)
            k2 = L3 * np.sin(q3)
            
            q2 = math.atan2(z_prime, r) - math.atan2(k2, k1)
            
            return [q1, q2, q3]
            
        return None # Analytical not supported for this DOF

    def inverse_kinematics_numerical(self, target_pos, current_angles, locked_joints=None):
        """
        Numerical IK using Scipy's optimization.
        Works for any DOF.
        """
        if locked_joints is None:
            locked_joints = [False] * self.dof
            
        def objective(angles):
            frames = self.forward_kinematics(angles)
            end_effector_pos = frames[-1][:3, 3]
            # Distance squared to target
            dist = np.sum((end_effector_pos - target_pos)**2)
            
            # Strong regularization to keep angles close to current (avoid joint flipping / folding)
            reg = 1.0 * np.sum((np.array(angles) - np.array(current_angles))**2)
            return dist + reg

        # Bounds force joint limits
        bounds = []
        for i in range(self.dof):
            if locked_joints[i]:
                bounds.append((current_angles[i], current_angles[i]))
            else:
                if i == 0:
                    bounds.append((-np.pi, np.pi)) # Base rotation is free
                else:
                    # Constrain structural joints to +/- 120 deg to strictly prevent self-intersection
                    bounds.append((-2.1, 2.1))
        
        result = minimize(
            objective, 
            current_angles, 
            method='L-BFGS-B', 
            bounds=bounds,
            options={'maxiter': 100, 'ftol': 1e-6}
        )
        
        return result.x.tolist()

    def ik(self, target_pos, current_angles, force_numerical=False, locked_joints=None):
        """Main IK solver combining analytical and numerical."""
        # Try analytical if suitable
        if not force_numerical and self.dof in [2, 3]:
            # We can only use analytical if the DH params match our assumptions
            # Let's say we check if alpha matches our typical 2/3 DOF setup
            if self.dof == 2 and self.dh_params[0]['alpha'] == 0:
                res = self.inverse_kinematics_analytical(target_pos, current_angles)
                if res is not None:
                    return res
            elif self.dof == 3 and math.isclose(abs(self.dh_params[0]['alpha']), np.pi/2, abs_tol=1e-3):
                res = self.inverse_kinematics_analytical(target_pos, current_angles)
                if res is not None:
                    return res
                    
        # Fallback to numerical
        return self.inverse_kinematics_numerical(target_pos, current_angles)

    def generate_workspace(self, samples=1000):
        points = []
        for _ in range(samples):
            angles = [np.random.uniform(-np.pi, np.pi) for _ in range(self.dof)]
            frames = self.forward_kinematics(angles)
            pos = frames[-1][:3, 3]
            points.append(pos.tolist())
        return points

class AbstractHardwareController:
    """
    Subclass this and implement `write_angles_to_motors` to control real robotics hardware
    (e.g., sending serial PWM signals to servos via Arduino or Raspberry Pi).
    """
    def __init__(self, arm: RobotArm):
        self.arm = arm
        self.current_angles = [0] * arm.dof
        
    def write_angles_to_motors(self, angles):
        raise NotImplementedError("You must implement this method to transmit data to your unique hardware setup.")
        
    def move_to_ik_target(self, x, y, z):
        """Calculates IK and seamlessly pushes the configuration directly to your hardware."""
        new_angles = self.arm.ik([x, y, z], self.current_angles)
        self.current_angles = new_angles
        self.write_angles_to_motors(new_angles)
        return new_angles
