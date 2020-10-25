import pybullet as p
import time
import pybullet_data
import math
import numpy as np
import random

def forwardKinematicsThreeJoints3D(L0, L1, L2, L3, L4, js):
  Ts = []

  c0 = math.cos(js[0])
  s0 = math.sin(js[0])
  c1 = math.cos(js[1])
  s1 = math.sin(js[1])
  c2 = math.cos(js[2])
  s2 = math.sin(js[2])
  c3 = math.cos(js[3])
  s3 = math.sin(js[3])

  Ts.append(np.matrix([[c0, -s0, 0, 0], [s0, c0, 0, 0], [0, 0, 1, L0], [0, 0, 0, 1]]))
  Ts.append(np.matrix([[1, 0, 0, 0], [0, c1, -s1, 0], [0, s1, c1, L1], [0, 0, 0, 1]]))
  Ts.append(np.matrix([[1, 0, 0, 0], [0, c2, -s2, 0], [0, s2, c2, L2], [0, 0, 0, 1]]))
  Ts.append(np.matrix([[1, 0, 0, 0], [0, c3, -s3, 0], [0, s3, c3, L3], [0, 0, 0, 1]]))
  Ts.append(np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, L4], [0, 0, 0, 1]]))

  TTs = [Ts[0], Ts[0]*Ts[1], Ts[0]*Ts[1]*Ts[2], Ts[0]*Ts[1]*Ts[2]*Ts[3], Ts[0]*Ts[1]*Ts[2]*Ts[3]*Ts[4]]

  xs1 = TTs[4][0,3] # Second Link
  ys1 = TTs[4][1,3]
  zs1 = TTs[4][2,3]
  print(xs1, ys1, zs1)
  return TTs


L0=0.02
L1=0.06
L2=0.06
L3=0.06
L4=0.085

phisicsClient = p.connect(p.GUI)

#URDF読み込み
p.loadURDF("plane.urdf", basePosition = [0, 0, 0])
p.loadURDF("box.urdf", basePosition = [0.10, 0.05, 0.5])
obUids = p.loadURDF("arm.urdf", basePosition = [0, 0, 0])

robot = obUids

gravId = p.addUserDebugParameter("gravity", -10, 10, -10)
jointIds = []
paramIds = []

p.setPhysicsEngineParameter(numSolverIterations=10)
p.changeDynamics(robot, -1, linearDamping=0, angularDamping=0)

#joint読み込み
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

#それぞれのjoint角度を定義
targetPos=[0 for i in range(len(paramIds))]
print("param:", len(paramIds))

p.setRealTimeSimulation(1)
while (1):
  p.setGravity(0, 0, p.readUserDebugParameter(gravId))
  for i in range(len(paramIds)):
    c = paramIds[i]
#手動操作
    if i!=5:
      targetPos[i] = p.readUserDebugParameter(c)
    else:
      targetPos[i] = -targetPos[i-1]
#自動微小移動
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

  forwardKinematicsThreeJoints3D(L0, L1, L2, L3, L4, targetPos[0:4])

  time.sleep(0.01)
