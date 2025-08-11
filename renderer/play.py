import numpy as np
from einops import rearrange
import mujoco
import mediapy as media
from tqdm import tqdm

# Setup mujoco
model = mujoco.MjModel.from_xml_path('model/scene.xml')
mj_data = mujoco.MjData(model)
# enable joint visualization option:
scene_option = mujoco.MjvOption()
scene_option.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True

# Load up the data
data = np.load('build/storage.npy') #storage.npy has the fields: state, timestamp and dt
state = data['state']
state = rearrange(state, '(frame dim) -> frame dim', dim=(model.nq + model.nv + model.nu))

print("after tensor transform", state.shape)

# Split into qpos, qvel, ctrl using model dimensions for robustness
qpos = state[:, :model.nq]
qvel = state[:, model.nq:model.nq+model.nv]
ctrl = state[:, model.nq+model.nv:]

print(qpos.shape, qvel.shape, ctrl.shape)

# Time management
dt_arr = data['dt']
print("dt mean and var:", dt_arr.mean(), dt_arr.var())

assert np.all(dt_arr > 0)
dt = dt_arr.mean()

camera = mujoco.MjvCamera()

body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
if body_id != -1:
    camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    camera.trackbodyid = body_id
    camera.distance = 3.0

# Render video
duration = int(data['timestamp'][-1] - data['timestamp'][0])
framerate = 60
time = 0

frames = []
mujoco.mj_resetData(model, mj_data)
with mujoco.Renderer(model) as renderer:
    for pos, vel, ctrl in tqdm(zip(qpos, qvel, ctrl), total=len(qpos), desc="Rendering frames"):
        mj_data.qpos[:] = pos
        mj_data.qvel[:] = vel
        mj_data.ctrl[:] = ctrl
        
        mujoco.mj_forward(model, mj_data)
        
        time = time + dt
        
        if len(frames) < time * framerate:
            renderer.update_scene(mj_data, camera=camera)
            pixels = renderer.render()
            frames.append(pixels)

media.write_video(f"build/trajectory.mp4", frames, fps=framerate)