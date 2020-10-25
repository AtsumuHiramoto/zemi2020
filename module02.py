import numpy as np
import math

def forwardKinematicsThreeJoints3D(L0,L1, L2, L3, L4, js):

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

  return TTs

def calcJointVelocity(vx, vy, vz, vtheta, js):
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

  J_ = np.linalg.inv(J)
  vjs = J_ * np.array([[vx], [vy], [vz], [vtheta]])
  print ("vjs:",vjs)
  return vjs

L0=1
L1=1
L2=1
L3=1
L4=1
calcJointVelocity(0, -5, 0, 1.57, [0, 0.01, 0.01, 0.01])

def inverseKinematicsByJ(x, y, z, sigma, theta, cjs):
  d_old = 1000.0
  js = np.array([[cjs[0]], [cjs[1]], [cjs[2]], [cjs[3]]])

  while True:
    csg = js[0,0]
    dsg = sigma - csg
    if dsg < 0.001 or d >= d_old:
      break
    

  while True:
    # まず，順運動学で手先位置を計算する
    T =forwardKinematicsThreeJoints3D(L0, L1, L2, L3, L4, js)
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
    d*


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
