import numpy as np
import math

def forwardKinematicsThreeJoints3D(L, js):
  Ts = []

  c0 = math.cos(js[0])
  s0 = math.sin(js[0])
  c1 = math.cos(js[1])
  s1 = math.sin(js[1])
  c2 = math.cos(js[2])
  s2 = math.sin(js[2])
  c3 = math.cos(js[3])
  s3 = math.sin(js[3])

  Ts.append(np.matrix([[c0, -s0, 0, 0], [s0, c0, 0, 0], [0, 0, 1, L[0]], [0, 0, 0, 1]]))
  Ts.append(np.matrix([[1, 0, 0, 0], [0, c1, -s1, 0], [0, s1, c1, L[1]], [0, 0, 0, 1]]))
  Ts.append(np.matrix([[1, 0, 0, 0], [0, c2, -s2, 0], [0, s2, c2, L[2]], [0, 0, 0, 1]]))
  Ts.append(np.matrix([[1, 0, 0, 0], [0, c3, -s3, 0], [0, s3, c3, L[3]], [0, 0, 0, 1]]))
  Ts.append(np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, L[4]], [0, 0, 0, 1]]))

  TTs = [Ts[0], Ts[0]*Ts[1], Ts[0]*Ts[1]*Ts[2], Ts[0]*Ts[1]*Ts[2]*Ts[3], Ts[0]*Ts[1]*Ts[2]*Ts[3]*Ts[4]]

  xs1 = TTs[4][0,3]
  ys1 = TTs[4][1,3]
  zs1 = TTs[4][2,3]
  #print(xs1, ys1, zs1)
  #return TTs
  return xs1, ys1,zs1

def move_P(K, theta, js):
  dtheta = theta-js
  if dtheta>0.001 or -0.001>dtheta:
    return K*dtheta
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
      [1, 1, 1]
  ])
  #print("J:",J.shape)
  #print(J)
  J_ = np.linalg.pinv(J)
  vjs = J_ * np.array([[vx], [vy], [vz], [vtheta]])
  #print (vjs)
  return vjs

def inverseKinematicsThreeJoints3D(L, x, y, z, theta, JS):

  dx=0.01
  dy=0.01
  dz=0.01
  dtheta=0.01
  js=JS
  js[0]=math.atan2(y,x)+math.pi/2
  #print(js[0])
  i=0
  while(True):
    #print("####",i,"####")
    ax, ay, az= forwardKinematicsThreeJoints3D(L, js)
    atheta=float(js[1]+js[2]+js[3])
    #print(ax,ay,az,atheta)
    vx=x-ax
    vy=y-ay
    vz=z-az
    vtheta=theta-atheta
    if math.sqrt(vx**2+vy**2+vz**2)<0.001 and vtheta<0.01:
      break
    if i==1000:
      print("解が収束しませんでした。")
      break
    dx=0.01 if vx>=0.01 else -0.01 if vx<=-0.01 else vx
    dy=0.01 if vy>=0.01 else -0.01 if vy<=-0.01 else vy
    dz=0.01 if vz>=0.01 else -0.01 if vz<=-0.01 else vz
    dtheta=0.01 if vtheta>=0.01 else -0.01 if vtheta<=-0.01 else vtheta
    #print(dx,dy,dz,dtheta)
    vjs=calcJointVelocity(L, dx, dy, dz, dtheta, js)
    #print(vjs)
    js[1]+=float(vjs[0])
    js[2]+=float(vjs[1])
    js[3]+=float(vjs[2])
    i+=1

  for j in range(len(js)):
    if js[j]>math.pi:
      js[j]=js[j]-int((js[j]+math.pi)/(2*math.pi))*2*math.pi
    elif js[j]<-math.pi:
      js[j]=js[j]-int((js[j]-math.pi)/(2*math.pi))*2*math.pi
  return js

"""
js=inverseKinematicsThreeJoints3D([0.2,0.6,0.6,0.6,0.85], 0.5, 0.5, 0.1, math.pi, [0, 0, 0, 0])
print(js)
print(forwardKinematicsThreeJoints3D([0.2,0.6,0.6,0.6,0.85], js))
"""
