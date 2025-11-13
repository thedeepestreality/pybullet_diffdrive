import pybullet as pb
import pybullet_data
import numpy as np
import time

# pb.connect(pb.GUI)
pb.connect(pb.DIRECT)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
# load the plane to stand onto
pb.loadURDF("plane.urdf")
# elevate robot so that the wheels are touching the plane
obj = pb.loadURDF("diff_drive.urdf.xml", 0, 0, 0.1)

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
logAlpha = np.zeros(sz)
logVel = np.zeros(sz)
logVelTheor = np.zeros(sz)

# desired angle
al_d = np.pi / 2.0
kp = 1.0
v_lin = 1.0 # m/s
wheel_rad = 0.05 # m

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
    orn = posAndOrn[1] # quaternion
    al = pb.getEulerFromQuaternion(orn)[2]
    logAlpha[idx] = al

    baseState = pb.getLinkState(obj, 0, computeLinkVelocity = 1)
    vel_vec = baseState[7]
    vel_lin = np.sqrt(vel_vec[0]**2 + vel_vec[1]**2)
    logVel[idx] = vel_lin
    wheel_state = pb.getJointStates(obj, motorIdx)
    wheel_vel = [wheel_state[0][1], wheel_state[1][1]]
    print(wheel_vel)
    logVelTheor[idx] = wheel_rad * np.pi * (wheel_vel[0] + wheel_vel[1])

    idx += 1
    # time.sleep(dt)

pb.disconnect()

# show XY plot
import matplotlib.pyplot as plt

plt.figure(1)
plt.title("XY plot")
plt.plot(logX, logY)
plt.axis("equal")

plt.figure(2)
plt.title("Alpha")
plt.plot(logTime, logAlpha)

plt.figure(3)
plt.title("Linear Velocity")
plt.plot(logTime, logVel, label='Sim')
plt.plot(logTime, logVelTheor, label='Theor')
plt.legend()
plt.show()
