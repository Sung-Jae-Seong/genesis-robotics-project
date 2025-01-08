######## Environment Setting
import os
os.environ['PYOPENGL_PLATFORM'] = 'glx' #before scene


######## Init Genesis / Generate scene
import genesis as gs
gs.init(backend=gs.cpu)

scene = gs.Scene(
    show_viewer=True,
    sim_options = gs.options.SimOptions(
        dt = 0.01,
    ),
)

plane = scene.add_entity(gs.morphs.Plane())
go2 = scene.add_entity(
    gs.morphs.URDF(
        file='urdf/go2/urdf/go2.urdf',
        pos  = (0.0, 0.0, 0.2),
    ),
)

scene.build()


######## Joint Definition
# FL_hip_joint      좌측 앞다리(몸통)
# FL_thigh_joint    
# FL_calf_joint     (말단)

# FR_hip_joint      우측 앞다리(몸통)
# FR_thigh_joint    
# FR_calf_joint     (말단)

# RL_hip_joint      좌측 뒷다리(몸통)
# RL_thigh_joint    
# RL_calf_joint     (말단)

# RR_hip_joint      우측 뒷다리(몸통)
# RR_thigh_joint    
# RR_calf_joint     (말단)

jnt_names = [
    'FL_hip_joint',
    'FL_thigh_joint',
    'FL_calf_joint',
    'FR_hip_joint',
    'FR_thigh_joint',
    'FR_calf_joint',
    'RL_hip_joint',
    'RL_thigh_joint',
    'RL_calf_joint',
    'RR_hip_joint',
    'RR_thigh_joint',
    'RR_calf_joint',
]
jnt_idx = [go2.get_joint(name).dof_idx_local for name in jnt_names]

while True:
    scene.step()