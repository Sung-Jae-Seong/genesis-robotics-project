import genesis as gs


class SceneManager:
    def __init__(self, dt=0.01, show_viewer=True):
        gs.init(backend=gs.cpu)
        self.dt = dt
        self.show_viewer = show_viewer
        self.scene = gs.Scene(
            show_viewer=self.show_viewer,
            sim_options=gs.options.SimOptions(dt=self.dt),
        )
        self.entities = []

    def add_entity(self, object):
        entity = self.scene.add_entity(object)
        self.entities.append(entity)
        return entity

    def build(self):
        self.scene.build()

    def step(self):
        self.scene.step()