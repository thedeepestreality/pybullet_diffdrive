import pybullet as pb
import pybullet_data
import numpy as np
import time

pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
# load the plane to stand onto
pb.loadURDF("plane.urdf")
# elevate robot so that the wheels are touching the plane
obj = pb.loadURDF("diff_drive.urdf", 0, 0, 0.1)

pb.setGravity(0, 0, -9.8)
# wheel order: [right, left]
motorIdx = [0, 1]
# const wheel velocities for test
vel = [6.0, 3.0]
maxTime = 5  # seconds
dt = 1 / 240  # seconds
logTime = np.arange(0, maxTime, dt)
sz = len(logTime)
logX = np.zeros(sz)
logY = np.zeros(sz)

# turn off castor wheel motor for free motion
pb.setJointMotorControl2(
    bodyIndex=obj,
    jointIndex=2,
    targetVelocity=0,
    controlMode=pb.VELOCITY_CONTROL,
    force=0,
)

# apply some wheel velocity
pb.setJointMotorControlArray(
    bodyIndex=obj,
    jointIndices=motorIdx,
    targetVelocities=vel,
    controlMode=pb.VELOCITY_CONTROL,
)

idx = 0
for t in logTime:
    pb.stepSimulation()
    posAndOrn = pb.getBasePositionAndOrientation(obj)
    pos = posAndOrn[0]
    logX[idx] = pos[0]
    logY[idx] = pos[1]
    idx += 1
    time.sleep(dt)

pb.disconnect()

# show XY plot
import matplotlib.pyplot as plt

plt.plot(logX, logY)
plt.axis("equal")
plt.show()
