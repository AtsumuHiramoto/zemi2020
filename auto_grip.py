import pybullet as p
import time
import pybullet_data
import math
import numpy as np
import random
import Pos_test as ps

phisicsClient = p.connect(p.GUI)

#物体の初期位置
x=0.07
y=0.03
z=0.01

L=[0.02, 0.06, 0.06, 0.06, 0.085]
theta = math.pi
K=[0.5,0.3,0.3,0.3,0.3]

#URDF読み込み
p.loadURDF("plane.urdf", basePosition = [0, 0, 0])
p.loadURDF("ball.urdf", basePosition = [x, y, z])
obUids = p.loadURDF("arm.urdf", basePosition = [0, 0, 0])

robot = obUids

gravId = p.addUserDebugParameter("gravity", -10, 10, -10)
jointIds = []
paramIds = []

p.setPhysicsEngineParameter(numSolverIterations=10)
p.changeDynamics(robot, -1, linearDamping=0, angularDamping=0)

#Joint読み込み
for j in range(p.getNumJoints(robot)):
  p.changeDynamics(robot, j, linearDamping=0, angularDamping=0)
  info = p.getJointInfo(robot, j)
  #print(info)
  jointName = info[1]
  jointType = info[2]
  if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
    jointIds.append(j)
    if j<6:
      paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, 0))
    else:
      paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), 0, 0.015, 0))

#各関節角を設定
#targetPos=[random.randint(0,100)*0.01 for i in range(len(paramIds))]
targetPos=[0 for i in range(len(paramIds))]
goalTheta=ps.inverseKinematicsThreeJoints3D(L, x, y, z, theta, [0, 0, 0, 0])

print("param:", len(paramIds))

p.setRealTimeSimulation(1)
while (1):
  p.setGravity(0, 0, p.readUserDebugParameter(gravId))
  for i in range(len(paramIds)):
    c = paramIds[i]
#手動操作
    """
    if i!=5:
      targetPos[i] = p.readUserDebugParameter(c)
    else:
      targetPos[i] = -targetPos[i-1]
    """
#微小移動
    #targetPos[i] +=-0.01

    p.setJointMotorControl2(robot, jointIds[i], p.POSITION_CONTROL, targetPos[i], force=5 * 240.)
    viewMatrix = p.computeViewMatrix(
      cameraEyePosition=[-1, 0, 0.2],
      cameraTargetPosition=[0, 0, 0],
      cameraUpVector=[0.1, 0, 0])

    projectionMatrix = p.computeProjectionMatrixFOV(
      fov=45.0,
      aspect=1.0,
      nearVal=0.1,
      farVal=3.1)

    width, height, rgbImg, depthImg, segImg = p.getCameraImage(
      width=224,
      height=224,
      viewMatrix=viewMatrix,
      projectionMatrix=projectionMatrix)

  #P制御
  for i in range(4):
    targetPos[i]+=ps.move_P(K[i], goalTheta[i], targetPos[i])
  if math.sqrt(sum((np.array(targetPos[0:4])-np.array(goalTheta))**2))<0.01:
    targetPos[4]+=ps.move_P(0.5, 0.006, targetPos[4])
    targetPos[5]=-targetPos[4]
  #print(targetPos)
  time.sleep(0.01)
