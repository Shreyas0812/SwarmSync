import numpy as np
import pybullet as p
import pybullet_data
import os
import time

class TrajectoryVisualizer:
    def __init__(self, trajectory_file='trajectories.txt'):
        # Parse trajectory file
        with open(trajectory_file, 'r') as f:
            lines = f.readlines()
        
        # Parse data
        data_rows = []
        for line in lines:
            values = [float(x) for x in line.strip().split()]
            data_rows.append(values)
        
        max_cols = max(len(row) for row in data_rows)
        M = np.zeros((len(data_rows), max_cols))
        for i, row in enumerate(data_rows):
            M[i, :len(row)] = row
        
        # Extract parameters
        self.N = int(M[0, 0])  # Number of agents
        self.N_cmd = int(M[0, 1])  # Number of commanded agents
        self.pmin = M[0, 2:5]  # Minimum bounds
        self.pmax = M[0, 5:8]  # Maximum bounds
        
        # Initial and target positions
        self.po = M[1:4, 0:self.N].T  # Shape: (N, 3)
        self.pf = M[4:7, 0:self.N_cmd].T  # Shape: (N_cmd, 3)
        
        # Extract trajectories
        start = 7
        num_timesteps = len(data_rows[start])
        self.pk = np.zeros((self.N_cmd, num_timesteps, 3))
        
        for i in range(self.N_cmd):
            self.pk[i, :, 0] = M[start + 3*i, :num_timesteps]  # x
            self.pk[i, :, 1] = M[start + 3*i + 1, :num_timesteps]  # y
            self.pk[i, :, 2] = M[start + 3*i + 2, :num_timesteps]  # z
        
        print(f"Loaded {self.N} drones, {self.N_cmd} commanded")
        print(f"Trajectory length: {num_timesteps} timesteps")
    
    def visualize_in_pybullet(self, gui=True, downsample=10):
        """Visualize trajectories in PyBullet"""
        # Initialize PyBullet
        if gui:
            self.client = p.connect(p.GUI)
        else:
            self.client = p.connect(p.DIRECT)
        
        # Set PyBullet data path to find URDFs
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        p.setGravity(0, 0, -9.8)
        p.setRealTimeSimulation(0)
        
        # Load ground plane (now PyBullet can find it)
        plane_id = p.loadURDF("plane.urdf")
        
        # Create simple drone representations using spheres (easier than loading URDFs)
        drone_ids = []
        colors = [
            [1, 0, 0, 1],  # Red
            [0, 1, 0, 1],  # Green
            [0, 0, 1, 1],  # Blue
            [1, 1, 0, 1],  # Yellow
        ]
        
        for i in range(self.N_cmd):
            # Create visual shape (sphere)
            visual_shape = p.createVisualShape(
                shapeType=p.GEOM_SPHERE,
                radius=0.1,
                rgbaColor=colors[i % len(colors)]
            )
            
            # Create collision shape
            collision_shape = p.createCollisionShape(
                shapeType=p.GEOM_SPHERE,
                radius=0.1
            )
            
            # Create multi-body
            drone_id = p.createMultiBody(
                baseMass=0.027,  # Crazyflie mass in kg
                baseCollisionShapeIndex=collision_shape,
                baseVisualShapeIndex=visual_shape,
                basePosition=self.po[i]
            )
            drone_ids.append(drone_id)
            
            # Add initial position marker (small sphere)
            p.addUserDebugLine(
                self.po[i] - [0.05, 0, 0],
                self.po[i] + [0.05, 0, 0],
                lineColorRGB=[0.5, 0.5, 0.5],
                lineWidth=2,
                lifeTime=0
            )
            p.addUserDebugLine(
                self.po[i] - [0, 0.05, 0],
                self.po[i] + [0, 0.05, 0],
                lineColorRGB=[0.5, 0.5, 0.5],
                lineWidth=2,
                lifeTime=0
            )
        
        # Add visual markers for targets (X marks)
        for i in range(self.N_cmd):
            # Red X markers for targets
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
        
        print(f"Replaying {timesteps} downsampled timesteps...")
        print("Close the PyBullet window to exit.")
        
        # Add time display
        time_text = p.addUserDebugText(
            text="Time: 0.00s",
            textPosition=[0, 0, 2.5],
            textColorRGB=[0, 0, 0],
            textSize=1.5,
            lifeTime=0
        )
        
        for t in range(timesteps):
            # Update drone positions
            for i in range(self.N_cmd):
                pos = pk_downsampled[i, t, :]
                p.resetBasePositionAndOrientation(
                    drone_ids[i],
                    pos,
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
                        lifeTime=0
                    )
            
            # Update time display
            sim_time = t * downsample * 0.01  # Assuming 0.01s per timestep
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
        print("The drones will remain at their final positions.")
        print("Close the PyBullet window to exit.")
        
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
    viz = TrajectoryVisualizer('trajectories.txt')
    viz.visualize_in_pybullet(gui=True, downsample=20)
