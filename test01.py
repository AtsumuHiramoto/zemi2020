import pybullet as p
import time
import pybullet_data

phisicsClient = p.connect(p.GUI)
#IDtr1 = p.loadSDF("kuka_iiwa/kuka_with_gripper.sdf")
p.loadURDF("plane.urdf", basePosition = [0, 0, 0])
obUids = p.loadURDF("arm.urdf", basePosition = [0, 0, 0])

robot = obUids

gravId = p.addUserDebugParameter("gravity", -10, 10, -10)
jointIds = []
paramIds = []

p.setPhysicsEngineParameter(numSolverIterations=10)
p.changeDynamics(robot, -1, linearDamping=0, angularDamping=0)

for j in range(p.getNumJoints(robot)):
  p.changeDynamics(robot, j, linearDamping=0, angularDamping=0)
  info = p.getJointInfo(robot, j)
  #print(info)
  jointName = info[1]
  jointType = info[2]
  if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
    jointIds.append(j)
    paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, 0))


p.setRealTimeSimulation(1)
while (1):
  p.setGravity(0, 0, p.readUserDebugParameter(gravId))
  for i in range(len(paramIds)):
    c = paramIds[i]
    targetPos = p.readUserDebugParameter(c)
    p.setJointMotorControl2(robot, jointIds[i], p.POSITION_CONTROL, targetPos, force=5 * 240.)

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

  time.sleep(0.01)
