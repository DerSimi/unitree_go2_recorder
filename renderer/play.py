import numpy as np
from einops import rearrange
import mujoco
import mediapy as media
from tqdm import tqdm
import matplotlib.pyplot as plt

# Setup mujoco
model = mujoco.MjModel.from_xml_path('model/scene.xml')
mj_data = mujoco.MjData(model)
# enable joint visualization option:
scene_option = mujoco.MjvOption()
scene_option.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True

# Load up the data
data = np.load('build/storage.npy') #storage.npy has the fields: state, timestamp and dt
state = data['state']
state = rearrange(state, '(frame dim) -> frame dim', dim=(model.nq + model.nv + model.nu + 8 * model.nu))

print("after tensor transform", state.shape)

# # Split into qpos, qvel, ctrl using model dimensions for robustness
nq = model.nq
nv = model.nv
nu = model.nu

# Indices
i = 0
qpos = state[:, i : i + nq]; i += nq
qvel = state[:, i : i + nv]; i += nv
ctrl = state[:, i : i + nu]; i += nu

# From low cmd
q      = state[:, i : i + nu]; i += nu
dq     = state[:, i : i + nu]; i += nu
tau    = state[:, i : i + nu]; i += nu
kp     = state[:, i : i + nu]; i += nu
kd     = state[:, i : i + nu]; i += nu

# From low state
tau_est = state[:, i : i + nu]; i += nu
q_raw   = state[:, i : i + nu]; i += nu
dq_raw  = state[:, i : i + nu]; i += nu

print(qpos.shape, qvel.shape, ctrl.shape)

state_array = rearrange(state[:, :model.nq+model.nv], 'frame posvel -> 1 frame posvel')
action_array = rearrange(state[:, model.nq+model.nv:], 'frame ctrl -> 1 frame ctrl')

print("State array shape", (state_array.shape))
print("Action array shape", (action_array.shape))

# # Time management
# dt_arr = data['dt']
# print("dt mean and var:", dt_arr.mean(), dt_arr.var())

# assert np.all(dt_arr > 0)
# dt = dt_arr.mean()

# print("qvel shape", qvel.shape)

# vel_xyz = qvel[:, :3]
# x_vel, y_vel, z_vel = vel_xyz.T 

# print(x_vel.shape)

# t = np.arange(len(x_vel)) * dt

# drei Subplots untereinander, shared x-axis
# fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

# axs[0].plot(t, x_vel, color='tab:blue')
# axs[0].set_ylabel('v_x [m/s]')
# axs[0].set_title('X-Velocity')

# axs[1].plot(t, y_vel, color='tab:orange')
# axs[1].set_ylabel('v_y [m/s]')
# axs[1].set_title('Y-Velocity')

# axs[2].plot(t, z_vel, color='tab:green')
# axs[2].set_ylabel('v_z [m/s]')
# axs[2].set_xlabel('Time [s]')
# axs[2].set_title('Z-Velocity')

# fig.tight_layout()
# plt.savefig('build/velocity_plots.png')
# plt.show()

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
        
        time = time + 0.01
        
        if len(frames) < time * framerate:
            renderer.update_scene(mj_data, camera=camera)
            pixels = renderer.render()
            frames.append(pixels)

media.write_video(f"build/trajectory.mp4", frames, fps=framerate)