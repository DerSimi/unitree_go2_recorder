import numpy as np
from einops import rearrange
import mujoco
import mediapy as media
from tqdm import tqdm  # Add this import

data = np.load('build/storage.npy')
state = data['state']

print("plain data", state.shape)

state = rearrange(state, '(frame dim) -> frame dim', dim=49)

# Bring data into right shape: 
# 49 = qpos + qvel + ctrl dim
print("after tensor transform", state.shape)

# Split into qpos, qvel, ctrl
qpos = state[:, :19]
qvel = state[:, 19:37]
ctrl = state[:, 37:]

print(qpos.shape, qvel.shape, ctrl.shape)

# Setup mujoco

model = mujoco.MjModel.from_xml_path('model/scene.xml')
data = mujoco.MjData(model)

# enable joint visualization option:
scene_option = mujoco.MjvOption()
scene_option.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True

# Simulate and display video.
frames = []
mujoco.mj_resetData(model, data)
with mujoco.Renderer(model) as renderer:
    for pos, vel, ctrl in tqdm(zip(qpos, qvel, ctrl), total=len(qpos), desc="Rendering frames"):
        data.qpos = pos
        data.qvel = vel
        data.ctrl = ctrl
        
        mujoco.mj_step(model, data)
        renderer.update_scene(data, scene_option=scene_option, camera="tracking")
        pixels = renderer.render()
        frames.append(pixels)

media.write_video(f"build/trajectory.mp4", frames, fps=60)