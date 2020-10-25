import numpy as np
import math

"""
L0=int(input('L0:'))
L1=int(input('L1:'))
L2=int(input('L2:'))
L3=int(input('L3:'))
L4=int(input('L4:'))
Theta=[]

for i in range(4):
  x=int(input('theta%d:'%i))
  Theta.append(math.pi*x/180)
"""

def forwardKinematicsThreeJoints3D(L0,L1, L2, L3, L4, js):
  """ 順運動学を解いて原点から各軸への変換行列を設定したTwoLinkRobot2Dクラスオブジェクトを作成する 
  :param js : 関節角度のリスト
  """
  Ts = []
  # sin, cosは繰り返し使うので，一度計算して変数に保存しておく
  c0 = math.cos(js[0])
  s0 = math.sin(js[0])
  c1 = math.cos(js[1])
  s1 = math.sin(js[1])
  c2 = math.cos(js[2])
  s2 = math.sin(js[2])
  c3 = math.cos(js[3])
  s3 = math.sin(js[3])
  # ここで変換行列を定義する．各リンクはデフォルトでz方向を向いているのでTzのみ使う
  # この変換は，移動してから回転なので注意
  # 回転軸によって回転行列が異なる
  Ts.append(np.matrix([[c0, -s0, 0, 0], [s0, c0, 0, 0], [0, 0, 1, L0], [0, 0, 0, 1]]))
  Ts.append(np.matrix([[1, 0, 0, 0], [0, c1, -s1, 0], [0, s1, c1, L1], [0, 0, 0, 1]]))
  Ts.append(np.matrix([[1, 0, 0, 0], [0, c2, -s2, 0], [0, s2, c2, L2], [0, 0, 0, 1]]))
  Ts.append(np.matrix([[1, 0, 0, 0], [0, c3, -s3, 0], [0, s3, c3, L3], [0, 0, 0, 1]]))
  Ts.append(np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, L4], [0, 0, 0, 1]]))

  # マトリクスを掛け算して，原点から各関節までの変換行列を得る
  TTs = [Ts[0], Ts[0]*Ts[1], Ts[0]*Ts[1]*Ts[2], Ts[0]*Ts[1]*Ts[2]*Ts[3], Ts[0]*Ts[1]*Ts[2]*Ts[3]*Ts[4]]

  return TTs

#print(forwardKinematicsThreeJoints3D(L0,L1, L2, L3, L4, Theta))

def getEndEffectorPoseMatrix(L0, L1, L2, L3, L4, theta):
  Ts = forwardKinematicsThreeJoints3D(L0, L1, L2, L3, L4, theta)
  return self.Ts[3]

def getEndEffectorPoseXYTheta(L0, L1, L2, L3, L4, theta):
  """
  Ts = forwardKinematicsThreeJoints3D(L0, L1, L2, L3, L4, theta)

  Theta  = math.atan2(Ts[3][1,0], Ts[3][0,0])
  print("Hand Pose:", Ts[3][0,2], Ts[3][1,2], Theta)

  return (Ts[3][0,2], Ts[3][1,2], Theta)
  """
  print(theta[0], theta[1]+theta[2]+theta[3], 0)
  return(theta[0], theta[1]+theta[2]+theta[3], 0)

def printFourLinkRobot3D(L0, L1, L2, L3, L4, theta):
  Ts = forwardKinematicsThreeJoints3D(L0, L1, L2, L3, L4, theta)
  xs1 = [0, Ts[0][0,3], Ts[1][0,3], Ts[2][0,3], Ts[3][0,3], Ts[4][0,3]] # Second Link
  ys1 = [0, Ts[0][1,3], Ts[1][1,3], Ts[2][1,3], Ts[3][1,3], Ts[4][1,3]]
  zs1 = [0, Ts[0][2,3], Ts[1][2,3], Ts[2][2,3], Ts[3][2,3], Ts[4][2,3]]
  print(xs1, "\n")
  print(ys1, "\n")
  print(zs1, "\n")

"""
Ts=forwardKinematicsThreeJoints3D(L0,L1, L2, L3, L4, Theta)
printFourLinkRobot3D(L0, L1, L2, L3, L4, Theta)
getEndEffectorPoseXYTheta(L0, L1, L2, L3, L4, Theta)
"""

def calcJointVelocity(vx, vy, vz, vtheta, js):
  #print(js)
  """ 目標手先位置Vx, Vy, Vthetaと現在関節角度jsを使う """
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
  print("J:",J.shape)
  J_ = np.linalg.inv(J)
  vjs = J_ * np.array([[vx], [vy], [vz], [vtheta]])
  print ("vjs:",vjs)
  return vjs

L0=1
L1=1
L2=1
L3=1
L4=1
calcJointVelocity(0.01, 0.01, 0.01, 0.01, [0.01, 0.01, 0.01, 0.01])

def inverseKinematicsByJ(x, y, z, theta, cjs):
  d_old = 1000.0
  js = np.array([[cjs[0]], [cjs[1]], [cjs[2]], [cjs[3]]])

  while True:
    # まず，順運動学で手先位置を計算する
    T =forwardKinematicsThreeJoints3D(L0,L1, L2, L3, L4, js)
    cx = T[4][0,3]
    cy = T[4][1,3]
    cz = T[4][2,3]
    csg = js[0,0]
    cth = js[1,0] + js[2,0] + js[3,0]

    # 現在の手先位置と目標手先位置の差分を取る
    dx = x - cx
    dy = y - cy
    dz = z - cz
    dsg = sigma - csg
    dth = theta - cth

    # この距離が小さいときは繰り返しを抜ける
    # 距離が減らなかった時もループを抜ける
    d = math.sqrt(dx**2 + dy**2 +dz**2)
    print('d = {}'.format(d))
    if d < 0.001 or d >= d_old:
      break


    # 手先の目標速度を得る
    vmax = 0.01
    vx  = -vmax if dx  < -vmax else dx if dx < vmax else vmax
    vy  = -vmax if dy  < -vmax else dy if dy < vmax else vmax
    vz  = -vmax if dz  < -vmax else dz if dz < vmax else vmax
    vsg = -vmax if dsg < -vmax else dsg if dsg < vmax else vmax
    vth = -vmax if dth < -vmax else dth if dth < vmax else vmax

    dt = 0.01

    # ヤコビ行列を使って目標手先速度を実現する各関節回転速度を得る
    #print(vx,vy,vth)
    vjs = calcJointVelocity(vx, vy, vz, vth, [js[0,0], js[1,0], js[2,0], js[3,0]])
    print(vjs)

    # 速度をかけて積分し，関節角度を更新する
    js = js + np.transpose(vjs) * dt

    d_old = d
  #print(d_old)

  return [js[0,0], js[1,0], js[2,0]]


x = 0.9
y = 0.99
z = 0.5
sigma = math.atan2(y,x)+math.pi/2
theta = -math.pi

#js0 = [1, -1.5, -0.2]
#js0 = [0.3, 1.0, -0.8]
js0 = [1, 1, 0, 0]
js1 = inverseKinematicsByJ(x, y, z, sigma, theta, js0)

print(js1)
