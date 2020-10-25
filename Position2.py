import numpy as np
import math

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

  xs1 = TTs[4][0,3]
  ys1 = TTs[4][1,3]
  zs1 = TTs[4][2,3]
  #print(xs1, ys1, zs1)
  return TTs

def getEndEffectorPoseMatrix(L0, L1, L2, L3, L4, theta):
  Ts = forwardKinematicsThreeJoints3D(L0, L1, L2, L3, L4, theta)
  return self.Ts[3]

def getEndEffectorPoseXYTheta(L0, L1, L2, L3, L4, theta):
  print(theta[0], theta[1]+theta[2]+theta[3], 0)
  return(theta[0], theta[1]+theta[2]+theta[3], 0)

def printFourLinkRobot3D(L0, L1, L2, L3, L4, theta):
  Ts = forwardKinematicsThreeJoints3D(L0, L1, L2, L3, L4, theta)
  xs1 = [0, Ts[0][0,3], Ts[1][0,3], Ts[2][0,3], Ts[3][0,3], Ts[4][0,3]]
  ys1 = [0, Ts[0][1,3], Ts[1][1,3], Ts[2][1,3], Ts[3][1,3], Ts[4][1,3]]
  zs1 = [0, Ts[0][2,3], Ts[1][2,3], Ts[2][2,3], Ts[3][2,3], Ts[4][2,3]]
  print(xs1, "\n")
  print(ys1, "\n")
  print(zs1, "\n")

def move_J0(K, sigma, j0):
  dsg = sigma-j0
  if dsg>0.01 or -0.01>dsg:
    return K*dsg
  else:
    return 0

def calcJointVelocity(L, vx, vy, vz, vtheta, js):
  #print(js)
  L0=L[0]
  L1=L[1]
  L2=L[2]
  L3=L[3]
  L4=L[4]
  """ 目標手先位置Vx, Vy, Vthetaと現在関節角度jsを使う """

  """
  J = np.matrix([
      [math.cos(js[0])*(L2*math.sin(js[1]) + L3*math.sin(js[1]+js[2]) + L4*math.sin(js[1]+js[2]+js[3])),
       math.sin(js[0])*(L2*math.cos(js[1]) + L3*math.cos(js[1]+js[2]) + L4*math.cos(js[1]+js[2]+js[3])),
       math.sin(js[0])*(L3*math.cos(js[1]+js[2]) + L4*math.cos(js[1]+js[2]+js[3])),
       math.sin(js[0])*(L4*math.cos(js[1]+js[2]+js[3])),
      ],
      [math.sin(js[0])*(L2*math.sin(js[1]) + L3*math.sin(js[1]+js[2]) + L4*math.sin(js[1]+js[2]+js[3])),
       -math.cos(js[0])*(L2*math.cos(js[1]) + L3*math.cos(js[1]+js[2]) + L4*math.cos(js[1]+js[2]+js[3])),
       -math.cos(js[0])*(L3*math.cos(js[1]+js[2]) + L4*math.cos(js[1]+js[2]+js[3])),
       -math.cos(js[0])*(L4*math.cos(js[1]+js[2]+js[3])),
      ],
      [0,
       -L2*math.sin(js[1]) - L3*math.sin(js[1]+js[2]) - L4*math.sin(js[1]+js[2]+js[3]),
       -L3*math.sin(js[1]+js[2]) - L4*math.sin(js[1]+js[2]+js[3]),
       -L4*math.sin(js[1]+js[2]+js[3]),
      ],
      [1, 1, 1, 1]
  ])
  """

  J = np.matrix([
      [math.sin(js[0])*(L2*math.cos(js[1]) + L3*math.cos(js[1]+js[2]) + L4*math.cos(js[1]+js[2]+js[3])),
       math.sin(js[0])*(L3*math.cos(js[1]+js[2]) + L4*math.cos(js[1]+js[2]+js[3])),
       math.sin(js[0])*(L4*math.cos(js[1]+js[2]+js[3])),
      ],
      [-math.cos(js[0])*(L2*math.cos(js[1]) + L3*math.cos(js[1]+js[2]) + L4*math.cos(js[1]+js[2]+js[3])),
       -math.cos(js[0])*(L3*math.cos(js[1]+js[2]) + L4*math.cos(js[1]+js[2]+js[3])),
       -math.cos(js[0])*(L4*math.cos(js[1]+js[2]+js[3])),
      ],
      [-L2*math.sin(js[1]) - L3*math.sin(js[1]+js[2]) - L4*math.sin(js[1]+js[2]+js[3]),
       -L3*math.sin(js[1]+js[2]) - L4*math.sin(js[1]+js[2]+js[3]),
       -L4*math.sin(js[1]+js[2]+js[3]),
      ],
      [1,1,1]
  ])
  #print("J:",J.shape)
  #print(J)
  J_ = np.linalg.pinv(J)
  vjs = J_ * np.array([[vx], [vy], [vz], [vtheta]])
  print (vjs)
  return vjs

def inverseKinematicsByJ(L, x, y, z, theta, cjs):
  d_old = 1000.0
  js = np.array([[cjs[0]], [cjs[1]], [cjs[2]], [cjs[3]]])

    # まず，順運動学で手先位置を計算する
  T =forwardKinematicsThreeJoints3D(L[0], L[1], L[2], L[3], L[4], js)
  cx = T[4][0,3]
  cy = T[4][1,3]
  cz = T[4][2,3]
  cth = js[1,0] + js[2,0] + js[3,0]

    # 現在の手先位置と目標手先位置の差分を取る
  dx = x - cx
  dy = y - cy
  dz = z - cz
  dth = theta - cth

    # この距離が小さいときは繰り返しを抜ける
    # 距離が減らなかった時もループを抜ける
  d = math.sqrt(dx**2 + dy**2 +dz**2)

  if d > 0.0001:

    # 手先の目標速度を得る
    vmax = 0.01
    vx  = -vmax if dx  < -vmax else dx if dx < vmax else vmax
    vy  = -vmax if dy  < -vmax else dy if dy < vmax else vmax
    vz  = -vmax if dz  < -vmax else dz if dz < vmax else vmax
    vth = -vmax if dth < -vmax else dth if dth < vmax else vmax

    # ヤコビ行列を使って目標手先速度を実現する各関節回転速度を得る
    #print("vx,vy,vz,vth:",vx,vy,vz,vth)
    vjs = calcJointVelocity(L, vx, vy, vz, vth, [js[0,0], js[1,0], js[2,0], js[3,0]])
    #print(vjs)

    # 速度をかけて積分し，関節角度を更新する
    t=0.1
    return [t*vjs[0,0], t*vjs[1,0], t*vjs[2,0]]

  else:
    return [0, 0, 0]

