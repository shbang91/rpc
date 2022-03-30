import pybullet as p
#import pybullet_data as pd
import time

p.connect(p.GUI)
dt = p.getPhysicsEngineParameters()['fixedTimeStep']
#p.setAdditionalSearchPath(pd.getDataPath())

#husky = p.loadURDF("husky/husky.urdf")
#husky = p.loadURDF("husky.urdf", flags=p.URDF_INITIALIZE_SAT_FEATURES)

col_shape_id = p.createCollisionShape(
    shapeType=p.GEOM_MESH,
    fileName="meshes/base_link.stl",
    flags=p.URDF_INITIALIZE_SAT_FEATURES #|p.GEOM_FORCE_CONCAVE_TRIMESH should only be used with fixed (mass=0) objects!
)

viz_shape_id = p.createVisualShape(
    shapeType=p.GEOM_MESH,
    fileName="meshes/base_link.stl",
)
body_id = p.createMultiBody(
    baseMass=1,
    baseCollisionShapeIndex=col_shape_id,
    baseVisualShapeIndex=viz_shape_id,
    basePosition=(0, 0, 0),
    baseOrientation=(0, 0, 0, 1),
)


while p.isConnected():
  p.stepSimulation()
  time.sleep(dt)