import numpy as np
import pybullet as p
import pybullet_data
import os
import time
import pkg_resources


class TrajectoryVisualizer:
    def __init__(self, trajectory_file='trajectories.txt'):
        # Parse trajectory file (your existing code)
        with open(trajectory_file, 'r') as f:
            lines = f.readlines()
        
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
        colors = [
            [1, 0, 0, 1],  # Red
            [0, 1, 0, 1],  # Green
            [0, 0, 1, 1],  # Blue
            [1, 1, 0, 1],  # Yellow
        ]
        
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
                lineColorRGB=[0.5, 0.5, 0.5],
                lineWidth=2,
                lifeTime=0
            )
        
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
        
        print(f"Replaying {timesteps} downsampled timesteps...")
        
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
    _root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    viz = TrajectoryVisualizer(os.path.join(_root, 'online_dmpc', 'cpp', 'results', 'trajectories.txt'))
    viz.visualize_in_pybullet(gui=True, downsample=20, use_urdf=True)
