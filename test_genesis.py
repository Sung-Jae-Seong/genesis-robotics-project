import os

os.environ['PYOPENGL_PLATFORM'] = 'glx'


###########################


import genesis as gs

gs.init(backend=gs.cpu)

scene = gs.Scene(show_viewer=True)
plane = scene.add_entity(gs.morphs.Plane())
franka = scene.add_entity(
    #gs.morphs.MJCF(file='xml/franka_emika_panda/panda.xml'),
    gs.morphs.URDF(file='./resource/m0609.urdf'),
)

scene.build()

#main loop
while True:
    scene.step()
