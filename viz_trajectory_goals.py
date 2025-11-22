import numpy as np
import pybullet as p
import pybullet_data
import os
import time
import sys
import pkg_resources
from tqdm import tqdm


class TrajectoryVisualizer:
    def __init__(self, trajectory_file='trajectories.txt', goal_file='goals.txt'):
        # Parse trajectory file (your existing code)
        with open(trajectory_file, 'r') as f:
            lines = f.readlines()
        
        self.goal_trajectories = None
        self.goals = None
        try:
            with open(goal_file, 'r') as f:
                goal_lines = f.readlines()
            
            self.goal_trajectories = self._parse_goal_file(goal_lines)
            if self.goal_trajectories is not None:
                self.goals = self.goal_trajectories[:, -1, :]

        except FileNotFoundError:
            print("Not able to open goal file")
        
        except Exception as e:
            print(f"Error reading goal file: {e}")

        data_rows = []
        for line in lines:
            values = [float(x) for x in line.strip().split()]
            data_rows.append(values)
        
        max_cols = max(len(row) for row in data_rows)
        M = np.zeros((len(data_rows), max_cols))
        for i, row in enumerate(data_rows):
            M[i, :len(row)] = row
        
        self.N = int(M[0, 0])
        self.N_cmd = int(M[0, 1])
        self.pmin = M[0, 2:5]
        self.pmax = M[0, 5:8]
        self.po = M[1:4, 0:self.N].T
        self.pf = M[4:7, 0:self.N_cmd].T
        
        start = 7
        num_timesteps = len(data_rows[start])
        self.pk = np.zeros((self.N_cmd, num_timesteps, 3))
        
        for i in range(self.N_cmd):
            self.pk[i, :, 0] = M[start + 3*i, :num_timesteps]
            self.pk[i, :, 1] = M[start + 3*i + 1, :num_timesteps]
            self.pk[i, :, 2] = M[start + 3*i + 2, :num_timesteps]
        
        print(f"Loaded {self.N} drones, {self.N_cmd} commanded")
        print(f"Trajectory length: {num_timesteps} timesteps")
    
    def _parse_goal_file(self, goal_lines):
        """Parse goal trajectories from goals.txt
        
        Format: 12 lines × 7500 columns
        - Lines 0-2: X, Y, Z coordinates for Drone 0 over 7500 timesteps
        - Lines 3-5: X, Y, Z coordinates for Drone 1 over 7500 timesteps
        - Lines 6-8: X, Y, Z coordinates for Drone 2 over 7500 timesteps
        - Lines 9-11: X, Y, Z coordinates for Drone 3 over 7500 timesteps
        
        Returns:
        --------
        goal_trajectories : np.array (N_drones, num_timesteps, 3)
        """
        data_rows = []
        for line in goal_lines:
            line = line.strip()
            if line:
                values = [float(x) for x in line.split()]
                data_rows.append(values)
        
        if not data_rows:
            return None
        
        M = np.array(data_rows)
        num_drones = M.shape[0] // 3
        num_timesteps = M.shape[1]
        
        goal_trajectories = np.zeros((num_drones, num_timesteps, 3))
        
        for i in range(num_drones):
            goal_trajectories[i, :, 0] = M[3*i, :]
            goal_trajectories[i, :, 1] = M[3*i + 1, :]
            goal_trajectories[i, :, 2] = M[3*i + 2, :]
        
        return goal_trajectories
    



    def visualize_in_pybullet(self, gui=True, downsample=10, use_urdf=True):
        """Visualize trajectories in PyBullet
        
        Parameters:
        -----------
        gui : bool
            Whether to use GUI mode
        downsample : int
            Downsampling factor for trajectory
        use_urdf : bool
            If True, load actual drone URDF. If False, use simple spheres.
        """
        # Initialize PyBullet
        if gui:
            self.client = p.connect(p.GUI)
        else:
            self.client = p.connect(p.DIRECT)
        
        # Set PyBullet data path for plane.urdf
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        p.setGravity(0, 0, -9.8)
        p.setRealTimeSimulation(0)
        
        # Load ground plane
        plane_id = p.loadURDF("plane.urdf")
        
        # Get path to Crazyflie URDF
        try:
            assets_path = pkg_resources.resource_filename('gym_pybullet_drones', 'assets')
            cf2x_urdf = os.path.join(assets_path, 'cf2x.urdf')
            print(f"Found drone URDF at: {cf2x_urdf}")
        except:
            print("Could not find gym-pybullet-drones assets. Using simple shapes.")
            use_urdf = False
        
        # Create drone representations
        drone_ids = []
        goal_drone_ids = []
        colors = [
            [1, 0, 0, 1],      # Red
            [0, 1, 0, 1],      # Green
            [0, 0, 1, 1],      # Blue
            [1, 1, 0, 1],      # Yellow
            [1, 0, 1, 1],      # Magenta
            [0, 1, 1, 1],      # Cyan
        ]
        
        # Actual drone models or spheres
        for i in range(self.N_cmd):
            if use_urdf and os.path.exists(cf2x_urdf):
                # Load actual Crazyflie URDF
                drone_id = p.loadURDF(
                    cf2x_urdf,
                    self.po[i],
                    p.getQuaternionFromEuler([0, 0, 0]),
                    flags=p.URDF_USE_INERTIA_FROM_FILE
                )
                print(f"Loaded drone {i} from URDF")
            else:
                # Fallback: Create visual sphere
                visual_shape = p.createVisualShape(
                    shapeType=p.GEOM_SPHERE,
                    radius=0.1,
                    rgbaColor=colors[i % len(colors)]
                )
                collision_shape = p.createCollisionShape(
                    shapeType=p.GEOM_SPHERE,
                    radius=0.1
                )
                drone_id = p.createMultiBody(
                    baseMass=0.027,
                    baseCollisionShapeIndex=collision_shape,
                    baseVisualShapeIndex=visual_shape,
                    basePosition=self.po[i]
                )
                print(f"Created sphere for drone {i}")
            
            drone_ids.append(drone_id)
            
            # Add initial position marker
            p.addUserDebugLine(
                self.po[i] - [0.05, 0, 0],
                self.po[i] + [0.05, 0, 0],
                lineColorRGB=[0, 0.8, 0],
                lineWidth=3,
                lifeTime=0
            )

        # Add goal position markers
        if self.goal_trajectories is not None:
            num_goal_drones = self.goal_trajectories.shape[0]
            
            # Create goal marker objects (semi-transparent spheres)
            for i in range(num_goal_drones):
                visual_shape = p.createVisualShape(
                    shapeType=p.GEOM_SPHERE,
                    radius=0.12,
                    rgbaColor=colors[i % len(colors)][:3] + [0.3]
                )
                goal_drone_id = p.createMultiBody(
                    baseMass=0,
                    baseVisualShapeIndex=visual_shape,
                    basePosition=self.goal_trajectories[i, 0, :]
                )
                goal_drone_ids.append(goal_drone_id)
            
            print(f"Created {num_goal_drones} goal trajectory markers")
        
        # Add target markers
        for i in range(self.N_cmd):
            p.addUserDebugLine(
                self.pf[i] - [0.1, 0.1, 0],
                self.pf[i] + [0.1, 0.1, 0],
                lineColorRGB=[1, 0, 0],
                lineWidth=3,
                lifeTime=0
            )
            p.addUserDebugLine(
                self.pf[i] - [0.1, -0.1, 0],
                self.pf[i] + [0.1, -0.1, 0],
                lineColorRGB=[1, 0, 0],
                lineWidth=3,
                lifeTime=0
            )
        
        # Replay trajectory
        pk_downsampled = self.pk[:, ::downsample, :]
        timesteps = pk_downsampled.shape[1]
        
        # Downsample goal trajectories to match
        goal_downsampled = None
        if self.goal_trajectories is not None:
            goal_len = self.goal_trajectories.shape[1]
            if goal_len == self.pk.shape[1]:
                goal_downsampled = self.goal_trajectories[:, ::downsample, :]
            else:
                # Resample to match
                indices = np.linspace(0, goal_len-1, timesteps).astype(int)
                goal_downsampled = self.goal_trajectories[:, indices, :]

        print(f"Replaying {timesteps} downsampled timesteps...")
        
        time_text = p.addUserDebugText(
            text="Time: 0.00s",
            textPosition=[0, 0, 2.5],
            textColorRGB=[0, 0, 0],
            textSize=1.5,
            lifeTime=0
        )
        
        for t in tqdm(range(timesteps), desc="Replaying trajectory", unit="step"):
            
            # Update drone positions
            for i in range(self.N_cmd):
                pos = pk_downsampled[i, t, :]
                p.resetBasePositionAndOrientation(
                    drone_ids[i],
                    pos,
                    p.getQuaternionFromEuler([0, 0, 0])
                )
            
            # Update goal positions if time-varying
            if goal_downsampled is not None:
                for i in range(min(len(goal_drone_ids), goal_downsampled.shape[0])):
                    goal_pos = goal_downsampled[i, t, :]
                    p.resetBasePositionAndOrientation(
                        goal_drone_ids[i],
                        goal_pos,
                        p.getQuaternionFromEuler([0, 0, 0])
                    )
            
            # Draw trajectory trails
            if t > 0:
                for i in range(self.N_cmd):
                    p.addUserDebugLine(
                        pk_downsampled[i, t-1, :],
                        pk_downsampled[i, t, :],
                        lineColorRGB=colors[i % len(colors)][:3],
                        lineWidth=2,
                        lifeTime=200
                    )
            
            # Draw goal trajectory trails if available
            if goal_downsampled is not None and t > 0:
                for i in range(min(self.N_cmd, goal_downsampled.shape[0])):
                    p.addUserDebugLine(
                        goal_downsampled[i, t-1, :],
                        goal_downsampled[i, t, :],
                        lineColorRGB=[1, 0.5, 0],  # Orange for goal trajectories
                        lineWidth=1,
                        lifeTime=0
                    )

            # Update time display
            sim_time = t * downsample * 0.01
            time_text = p.addUserDebugText(
                text=f"Time: {sim_time:.2f}s",
                textPosition=[0, 0, 2.5],
                textColorRGB=[0, 0, 0],
                textSize=1.5,
                lifeTime=0,
                replaceItemUniqueId=time_text
            )
            
            p.stepSimulation()
            if gui:
                time.sleep(0.05)
        
        print("\nTrajectory replay complete!")

        # Compute and display final errors
        final_positions = self.pk[:, -1, :]
        if self.goals is not None:
            errors = np.linalg.norm(final_positions - self.goals[:self.N_cmd], axis=1)
            print("\nFinal tracking errors (actual vs goal trajectories):")
            for i, err in enumerate(errors):
                print(f"  Drone {i}: {err:.4f} m")
            print(f"  Mean error: {np.mean(errors):.4f} m")
            print(f"  Max error: {np.max(errors):.4f} m")
        
        # Also compute errors vs pf
        errors_pf = np.linalg.norm(final_positions - self.pf, axis=1)
        print("\nFinal tracking errors (actual vs trajectory endpoints pf):")
        for i, err in enumerate(errors_pf):
            print(f"  Drone {i}: {err:.4f} m")
        
        # Keep window open
        if gui:
            try:
                while True:
                    p.stepSimulation()
                    time.sleep(0.01)
            except KeyboardInterrupt:
                print("\nExiting...")
        
        p.disconnect()


if __name__ == "__main__":
    # viz = TrajectoryVisualizer('trajectories.txt', 'goals.txt')
    # viz = TrajectoryVisualizer('scenario_1_trajectories.txt', 'scenario_1_goals.txt')
    # viz = TrajectoryVisualizer('scenario_2_trajectories.txt', 'scenario_2_goals.txt')
    viz = TrajectoryVisualizer('scenario_3_trajectories.txt', 'scenario_3_goals.txt')
    viz.visualize_in_pybullet(gui=True, downsample=20, use_urdf=True)
