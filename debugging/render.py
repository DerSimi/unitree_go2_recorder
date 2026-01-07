import mujoco
import mediapy as media
from tqdm import tqdm

from data_loader import DataLoader

# Load data
data_loader = DataLoader(name='test')
data_loader.print_debug()

model = data_loader.get_mj_model()
mj_data = mujoco.MjData(model)
# enable joint visualization option:
scene_option = mujoco.MjvOption()
scene_option.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True

qpos = data_loader.get_qpos()
qvel = data_loader.get_qvel()
ctrl = data_loader.get_ctrl()

camera = mujoco.MjvCamera()

body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
if body_id != -1:
    camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    camera.trackbodyid = body_id
    camera.distance = 3.0

# Render video
duration = data_loader.get_duration()
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
        
        time = time + 0.002
        
        if len(frames) < time * framerate:
            renderer.update_scene(mj_data, camera=camera)
            pixels = renderer.render()
            frames.append(pixels)

media.write_video(f"output/trajectory.mp4", frames, fps=framerate)
print("Video written to output directory!")