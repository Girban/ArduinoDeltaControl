# program for kinematic analysis

import numpy as np
import math
def call_inverse(x, y, z, Al, Bi, Le, Lf):
    al = Al
    bi = Bi
    le = Le
    lf = Lf
    xo = x
    yo = y
    zo = z
    a1 = (xo * xo + yo * yo + zo * zo + lf * lf - le * le - al * al + bi * bi - 2 * bi * yo) / (2 * zo)
    b1 = (bi - al - yo) / zo
    con1 = (al + a1 * b1) * (al + a1 * b1) - (1 + b1 * b1) * (al * al + a1 * a1 - lf * lf)
    if (con1 < 0):
        print(" The chosen point is beyond the work space of the robot for the selected lengths of the parameters.\n")
    else:
        y1a = (-(al + a1 * b1) + math.sqrt(con1)) / (1 + b1 * b1)
        y1b = (-(al + a1 * b1) - math.sqrt(con1)) / (1 + b1 * b1)
    if (y1a < y1b):
        z1a = a1 + b1 * y1a
        th1 = (180 / math.pi) * (math.atan(z1a / abs(y1a + al)))
    else:
        z1b = a1 + b1 * y1b
        th1 = (180 / math.pi) * (math.atan(z1b / abs(y1b + al)))
        xo2 = (-xo + math.sqrt(3) * yo) / 2
        yo2 = (-(math.sqrt(3) * xo + yo)) / 2
        zo2 = zo
        a2 = (xo2 * xo2 + yo2 * yo2 + zo2 * zo2 + lf * lf - le * le - al * al + bi * bi - 2 * bi * yo2) / (2 * zo2)
        b2 = (bi - al - yo2) / zo2
        con2 = (al + a2 * b2) * (al + a2 * b2) - (1 + b2 * b2) * (al * al + a2 * a2 - lf * lf)
    if (con2 < 0):
        messagebox.showinfo("Error",
                            " The chosen point is beyond the work space of the robot for the selected lengths of the parameters.")
        print("The chosen point is beyond the work space of the robot for the selected lengths of the parameters.\n")
    else:
        y2a = (-(al + a2 * b2) + math.sqrt(con2)) / (1 + b2 * b2)
        y2b = (-(al + a2 * b2) - math.sqrt(con2)) / (1 + b2 * b2)
    if (y2a < y2b):
        z2a = a2 + b2 * y2a
        th2 = (180 / math.pi) * (math.atan(z2a / abs(y2a + al)))
    else:
        z2b = a2 + b2 * y2b
        th2 = (180 / math.pi) * (math.atan(z2b / abs(y2b + al)))
        xo3 = (-(xo + math.sqrt(3) * yo)) / 2
        yo3 = (math.sqrt(3) * xo - yo) / 2
        zo3 = zo
        a3 = (xo3 * xo3 + yo3 * yo3 + zo3 * zo3 + lf * lf - le * le - al * al + bi * bi - 2 * bi * yo3) / (2 * zo3)
        b3 = (bi - al - yo3) / zo3
    con3 = (al + a3 * b3) * (al + a3 * b3) - (1 + b3 * b3) * (al * al + a3 * a3 - lf * lf)
    if (con3 < 0):
        messagebox.showinfo("Error",
                            " The chosen point is beyond the work space of the robot for the selected lengths of the parameters.")
        print("The chosen point is beyond the work space of the robot for the selected lengths of the parameters.\n")
    else:
        y3a = (-(al + a3 * b3) + math.sqrt(con3)) / (1 + b3 * b3)
        y3b = (-(al + a3 * b3) - math.sqrt(con3)) / (1 + b3 * b3)
    if (y3a < y3b):
        z3a = a3 + b3 * y3a
        th3 = (180 / math.pi) * (math.atan(z3a / abs(y3a + al)))
    else:
        z3b = a3 + b3 * y3b
        th3 = (180 / math.pi) * (math.atan(z3b / abs(y3b + al)))
    return th1, th2, th3

def call_forward(th1,th2,th3,al,bi,le,lf):
    ga=al-bi
    V=-2*lf*np.sin(np.radians(th1))
    W=le*le-lf*lf-al*al-bi*bi+2*al*bi+2*bi*lf*np.cos(np.radians(th1))-2*al*lf*np.cos(np.radians(th1))
    A1=np.sqrt(3)*(ga+lf*np.cos(np.radians(th2)))
    B1=(3*ga+2*lf*np.cos(np.radians(th1))+lf*np.cos(np.radians(th2)))
    C1=2*lf*(np.sin(np.radians(th2))-np.sin(np.radians(th1)))
    D1=-2*ga*lf*(np.cos(np.radians(th1))-np.cos(np.radians(th2)))
    A2=np.sqrt(3)*(-ga-lf*np.cos(np.radians(th3)))
    B2=(3*ga+2*lf*np.cos(np.radians(th1))+lf*np.cos(np.radians(th3)))
    C2=2*lf*(np.sin(np.radians(th3))-np.sin(np.radians(th1)))
    D2=-2*ga*lf*(np.cos(np.radians(th1))-np.cos(np.radians(th3)))
    A3=np.sqrt(3)*(-2*ga-lf*np.cos(np.radians(th2))-lf*np.cos(np.radians(th3)))
    B3=lf*(np.cos(np.radians(th3))-np.cos(np.radians(th2)))
    C3=2*lf*(np.sin(np.radians(th3))-np.sin(np.radians(th2)))
    D3=-2*ga*lf*(np.cos(np.radians(th2))-np.cos(np.radians(th3)))
    V=-2*lf*np.sin(np.radians(th1))
    W=le*le-lf*lf-al*al-bi*bi+2*al*bi+2*bi*lf*np.cos(np.radians(th1))-2*al*lf*np.cos(np.radians(th1))
    P=(D1*A2-D2*A1)/(B1*A2-B2*A1)
    Q=(C2*A1-C1*A2)/(B1*A2-B2*A1)
    R=(D3-P*B3)/A3
    S=-(C3+Q*B3)/A3
    U=2*(ga+lf*np.cos(np.radians(th1)))
    A=1+S*S+Q*Q
    B=2*R*S+2*P*Q+U*Q+V
    C=R*R+P*P+U*P-W
    zo=(-B-np.sqrt(B*B-4*A*C))/(2*A)
    xo=R+S*zo
    yo=P+Q*zo
    return xo,yo,zo