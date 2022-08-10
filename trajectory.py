# program for trajectory planning

global x0,y0,z0
global xd,yd,zd
import numpy as np
import math
import Kinematics
global X0 , Y0 , Z0
th1h=0;th2h=0;th3h=0;
thh=[th1h,th2h,th3h];
al=100;bl=50;le=280;lf=100;
[X0,Y0,Z0]=Kinematics.call_forward(th1h,th2h,th3h,al,bl,le,lf);

#path1
def path1(Xp,Yp,Zp):
    XA = Xp; YA = Yp; ZA = Z0;
    thA = Kinematics.call_inverse(XA, YA, ZA, al, bl, le, lf)
    thp = Kinematics.call_inverse(Xp, Yp, Zp, al, bl, le, lf)
    thi1 = np.array([thh, thA, thp])
    T = [0, 2, 4];
    b = 10; p=2;
    t = np.zeros([p, b])
    C1 = np.array(np.zeros([4, p]))
    C2 = np.array(np.zeros([4, p]))
    C3 = np.array(np.zeros([4, p]))
    th1c = np.zeros([p, b]);
    th2c = np.zeros([p, b]);
    th3c = np.zeros([p, b])
    v1c = np.zeros([p, b]);
    v2c = np.zeros([p, b]);
    v3c = np.zeros([p, b]);
    a1c = np.zeros([p, b]);
    a2c = np.zeros([p, b]);
    a3c = np.zeros([p, b])
    for j in range(2):
        [th1ji, th2ji, th3ji] = thi1[j, :]
        [th1jf, th2jf, th3jf] = thi1[j + 1, :]
        A = np.array([[1, T[j], math.pow(T[j], 2), math.pow(T[j], 3)],
                      [0, 1, 2 * T[j], 3 * math.pow(T[j], 2)],
                      [1, T[j + 1], math.pow(T[j + 1], 2), math.pow(T[j + 1], 3)],
                      [0, 1, 2 * T[j + 1], 3 * math.pow(T[j + 1], 2)]])
        B1 = np.array([[th1ji], [0], [th1jf], [0]]);
        X1 = np.dot(np.linalg.inv(A), B1)
        C1[:, j] = np.reshape(X1, (1, 4))
        B2 = np.array([[th2ji], [0], [th2jf], [0]]);
        X2 = np.dot(np.linalg.inv(A), B2)
        C2[:, j] = np.reshape(X2, (1, 4))
        B3 = np.array([[th3ji], [0], [th3jf], [0]]);
        X3 = np.dot(np.linalg.inv(A), B3)
        C3[:, j] = np.reshape(X3, (1, 4))
        t1 = np.linspace(T[j], T[j + 1], b);
        th11 = C1[0, j] + C1[1, j] * t1 + C1[2, j] * np.power(t1, 2) + C1[3, j] * np.power(t1, 3)
        th21 = C2[0, j] + C2[1, j] * t1 + C2[2, j] * np.power(t1, 2) + C2[3, j] * np.power(t1, 3)
        th31 = C3[0, j] + C3[1, j] * t1 + C3[2, j] * np.power(t1, 2) + C3[3, j] * np.power(t1, 3)
        v11 = C1[1, j] + 2 * C1[2, j] * t1 + 3 * np.power(t1, 2) * C1[3, j]
        v21 = C2[1, j] + 2 * C2[2, j] * t1 + 3 * np.power(t1, 2) * C2[3, j]
        v31 = C3[1, j] + 2 * C3[2, j] * t1 + 3 * np.power(t1, 2) * C3[3, j]
        th1c[j, :] = th11; th2c[j, :] = th21; th3c[j, :] = th31;
        t[j, :] = t1;
        v1c[j, :] = v11; v2c[j, :] = v21; v3c[j, :] = v31;
    th1c = np.reshape(th1c, (b * p, 1));
    th2c = np.reshape(th2c, (b * p, 1));
    th3c = np.reshape(th3c, (b * p, 1))
    t = np.reshape(t, (b * p, 1))
    v1c = np.reshape(v1c, (b * p, 1));
    v2c = np.reshape(v2c, (b * p, 1));
    v3c = np.reshape(v3c, (b * p, 1));
    theta=np.array([th1c,th2c,th3c]);
    vel=np.array([v1c,v2c,v3c])
    return t, theta, vel

#path2
def path2(Xp,Yp,Zp):
    XD = 100; YD = 100; ZD = Z0 - 20;
    XA = Xp; YA = Yp; ZA = Z0+20;
    thA = Kinematics.call_inverse(XA, YA, ZA, al, bl, le, lf)
    thp = Kinematics.call_inverse(Xp, Yp, Zp, al, bl, le, lf)
    XB = XD; YB = YD; ZB = Z0+20;
    thB = Kinematics.call_inverse(XB, YB, ZB, al, bl, le, lf)
    thD = Kinematics.call_inverse(XD, YD, ZD, al, bl, le, lf)
    T = [4, 6, 8, 10]
    b=10;p=3;
    thi2 = np.array([thp, thA, thB, thD])
    t = np.zeros([p, b])
    C1 = np.array(np.zeros([4, p]))
    C2 = np.array(np.zeros([4, p]))
    C3 = np.array(np.zeros([4, p]))
    th1c = np.zeros([p, b]);
    th2c = np.zeros([p, b]);
    th3c = np.zeros([p, b])
    v1c = np.zeros([p, b]);
    v2c = np.zeros([p, b]);
    v3c = np.zeros([p, b]);
    for j in range(3):
        [th1ji, th2ji, th3ji] = thi2[j, :]
        [th1jf, th2jf, th3jf] = thi2[j + 1, :]
        A = np.array([[1, T[j], math.pow(T[j], 2), math.pow(T[j], 3)],
                      [0, 1, 2 * T[j], 3 * math.pow(T[j], 2)],
                      [1, T[j + 1], math.pow(T[j + 1], 2), math.pow(T[j + 1], 3)],
                      [0, 1, 2 * T[j + 1], 3 * math.pow(T[j + 1], 2)]])
        B1 = np.array([[th1ji], [0], [th1jf], [0]]);
        X1 = np.dot(np.linalg.inv(A), B1)
        C1[:, j] = np.reshape(X1, (1, 4))
        B2 = np.array([[th2ji], [0], [th2jf], [0]]);
        X2 = np.dot(np.linalg.inv(A), B2)
        C2[:, j] = np.reshape(X2, (1, 4))
        B3 = np.array([[th3ji], [0], [th3jf], [0]]);
        X3 = np.dot(np.linalg.inv(A), B3)
        C3[:, j] = np.reshape(X3, (1, 4))
        t1 = np.linspace(T[j], T[j + 1], b);
        th11 = C1[0, j] + C1[1, j] * t1 + C1[2, j] * np.power(t1, 2) + C1[3, j] * np.power(t1, 3)
        th21 = C2[0, j] + C2[1, j] * t1 + C2[2, j] * np.power(t1, 2) + C2[3, j] * np.power(t1, 3)
        th31 = C3[0, j] + C3[1, j] * t1 + C3[2, j] * np.power(t1, 2) + C3[3, j] * np.power(t1, 3)
        v11 = C1[1, j] + 2 * C1[2, j] * t1 + 3 * np.power(t1, 2) * C1[3, j]
        v21 = C2[1, j] + 2 * C2[2, j] * t1 + 3 * np.power(t1, 2) * C2[3, j]
        v31 = C3[1, j] + 2 * C3[2, j] * t1 + 3 * np.power(t1, 2) * C3[3, j]
        th1c[j, :] = th11;
        th2c[j, :] = th21;
        th3c[j, :] = th31;
        t[j, :] = t1;
        v1c[j, :] = v11;
        v2c[j, :] = v21;
        v3c[j, :] = v31;
    th1c = np.reshape(th1c, (b * p, 1));
    th2c = np.reshape(th2c, (b * p, 1));
    th3c = np.reshape(th3c, (b * p, 1))
    t = np.reshape(t, (b * p, 1))
    v1c = np.reshape(v1c, (b * p, 1));
    v2c = np.reshape(v2c, (b * p, 1));
    v3c = np.reshape(v3c, (b * p, 1));
    theta = np.array([th1c, th2c, th3c]);
    vel = np.array([v1c, v2c, v3c])
    return t, theta, vel

#path3
def path3(Xp,Yp,Zp):
    XD = 100; YD = 100; ZD = Z0 - 20;
    XB = XD; YB = YD; ZB = Z0+20;
    thB = Kinematics.call_inverse(XB, YB, ZB, al, bl, le, lf)
    thD = Kinematics.call_inverse(XD, YD, ZD, al, bl, le, lf)
    T = [10, 12, 14]
    thi3 = np.array([thD, thB, thh])
    b = 10; p = 2;
    t = np.zeros([p, b])
    C1 = np.array(np.zeros([4, p]))
    C2 = np.array(np.zeros([4, p]))
    C3 = np.array(np.zeros([4, p]))
    th1c = np.zeros([p, b]);
    th2c = np.zeros([p, b]);
    th3c = np.zeros([p, b])
    v1c = np.zeros([p, b]);
    v2c = np.zeros([p, b]);
    v3c = np.zeros([p, b]);
    for j in range(p):
        [th1ji, th2ji, th3ji] = thi3[j, :]
        [th1jf, th2jf, th3jf] = thi3[j + 1, :]
        A = np.array([[1, T[j], math.pow(T[j], 2), math.pow(T[j], 3)],
                      [0, 1, 2 * T[j], 3 * math.pow(T[j], 2)],
                      [1, T[j + 1], math.pow(T[j + 1], 2), math.pow(T[j + 1], 3)],
                      [0, 1, 2 * T[j + 1], 3 * math.pow(T[j + 1], 2)]])
        B1 = np.array([[th1ji], [0], [th1jf], [0]]);
        X1 = np.dot(np.linalg.inv(A), B1)
        C1[:, j] = np.reshape(X1, (1, 4))
        B2 = np.array([[th2ji], [0], [th2jf], [0]]);
        X2 = np.dot(np.linalg.inv(A), B2)
        C2[:, j] = np.reshape(X2, (1, 4))
        B3 = np.array([[th3ji], [0], [th3jf], [0]]);
        X3 = np.dot(np.linalg.inv(A), B3)
        C3[:, j] = np.reshape(X3, (1, 4))
        t1 = np.linspace(T[j], T[j + 1], b);
        th11 = C1[0, j] + C1[1, j] * t1 + C1[2, j] * np.power(t1, 2) + C1[3, j] * np.power(t1, 3)
        th21 = C2[0, j] + C2[1, j] * t1 + C2[2, j] * np.power(t1, 2) + C2[3, j] * np.power(t1, 3)
        th31 = C3[0, j] + C3[1, j] * t1 + C3[2, j] * np.power(t1, 2) + C3[3, j] * np.power(t1, 3)
        v11 = C1[1, j] + 2 * C1[2, j] * t1 + 3 * np.power(t1, 2) * C1[3, j]
        v21 = C2[1, j] + 2 * C2[2, j] * t1 + 3 * np.power(t1, 2) * C2[3, j]
        v31 = C3[1, j] + 2 * C3[2, j] * t1 + 3 * np.power(t1, 2) * C3[3, j]
        th1c[j, :] = th11;
        th2c[j, :] = th21;
        th3c[j, :] = th31;
        t[j, :] = t1;
        v1c[j, :] = v11;
        v2c[j, :] = v21;
        v3c[j, :] = v31;
    th1c = np.reshape(th1c, (b * p, 1));
    th2c = np.reshape(th2c, (b * p, 1));
    th3c = np.reshape(th3c, (b * p, 1))
    t = np.reshape(t, (b * p, 1))
    v1c = np.reshape(v1c, (b * p, 1));
    v2c = np.reshape(v2c, (b * p, 1));
    v3c = np.reshape(v3c, (b * p, 1));
    theta = np.array([th1c, th2c, th3c]);
    vel = np.array([v1c, v2c, v3c])
    return t, theta, vel

#combine path
def compath(xp,yp,zp):
    [t1,thp,vp]=np.array(path1(xp,yp,zp))
    [t2,thpl,vpl]=np.array(path2(xp,yp,zp))
    [t3,thr,vr]=np.array(path3(xp,yp,zp))
    [th1p,th2p,th3p]=thp; [th1pl,th2pl,th3pl]=thpl; [th1r,th2r,th3r]=thr;
    [v1p,v2p,v3p]=vp; [v1pl,v2pl,v3pl]=vpl; [v1r,v2r,v3r]=vr;
    t=np.append(t1,t2)
    t=np.append(t,t3)
    th1=np.append(th1p,th1pl); th2=np.append(th2p,th2pl);th3=np.append(th3p,th3pl)
    v1=np.append(v1p,v1pl); v2=np.append(v2p,v2pl); v3=np.append(v3p,v3pl)
    th1=np.append(th1,th1r); th2=np.append(th2,th2r); th3=np.append(th3,th3r)
    v1=np.append(v1,v1r); v2=np.append(v2,v2r); v3=np.append(v3,v3r)
    v=np.array([v1,v2,v3])
    th=np.array([th1,th2,th3])
    return t,th,v
def steps(xp,yp,zp):
    [t,pos,v]=compath(xp,yp,zp);
    [th1,th2,th3]=pos;
    r, = th1.shape
    step1 = np.zeros(r - 1);
    step2 = np.zeros(r - 1);
    step3 = np.zeros(r - 1);
    for i in range(r - 1):
        step1[i] = th1[i + 1] - th1[i]
        step2[i] = th2[i + 1] - th2[i]
        step3[i] = th3[i + 1] - th3[i]
    return step1, step2, step3
def steppath(xp,yp,zp,op):
    if(op=="pick"):
        [t, pos, v] = np.array(path1(xp, yp, zp))
    elif(op=="place"):
        [t, pos, v] = np.array(path2(xp, yp, zp))
    else:
        [t, pos, v] = np.array(path3(xp, yp, zp))
    [th1, th2, th3] = pos;
    r,c = th1.shape
    step1 = np.zeros(r - 1);
    step2 = np.zeros(r - 1);
    step3 = np.zeros(r - 1);
    for i in range(r - 1):
        step1[i] = th1[i + 1] - th1[i]
        step2[i] = th2[i + 1] - th2[i]
        step3[i] = th3[i + 1] - th3[i]
    return step1, step2, step3
