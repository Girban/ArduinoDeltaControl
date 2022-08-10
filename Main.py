# Main program

#importing necessary modules
import numpy as np
import ip2
import tkinter as tk
import tkinter.font as font
import cv2
import ipn
import time
from PIL import Image, ImageTk
import coordinate
from tkinter import ttk
import Ard_pycmd
import ard_py
import threading
import coordinate_transformation
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import Kinematics
import trajectory
global thrd

Ard_pycmd.step_up(0,0,0,1)
th1h = 0; th2h = 0; th3h = 0;
posh = np.array([th1h, th2h, th3h])
al = 100; bl = 50; le = 280; lf = 100;
[x0, y0, z0] = Kinematics.call_forward(th1h, th2h, th3h, al, bl, le, lf);
[t,th,v]=trajectory.compath(60,60,z0-40)
cap=cv2.VideoCapture(0)
[th1c,th2c,th3c]=th
[v1c,v2c,v3c]=v
global lmain
global lmain2
global figure
root = tk.Tk()
root.geometry("1500x750")
root.title("User Interface for PICK and PLACE Robot")
iv = tk.IntVar()
sv1 = tk.IntVar()
sv2 = tk.IntVar()
sv3 = tk.IntVar()
sv4= tk.IntVar()
sv4.set("Off")
url1 = tk.StringVar()
url2 = tk.StringVar()
ur1="http://192.168.43.1:8080/shot.jpg";
ur2="http://192.168.43.1:8080/shot.jpg";
url1.set(ur1);
url2.set(ur2);
url = tk.StringVar()
status = tk.StringVar()
status.set("nothing")
cl = tk.StringVar()
cenx = tk.DoubleVar()
olx=tk.DoubleVar()
ceny = tk.DoubleVar()
oly=tk.DoubleVar()
CenZ = tk.DoubleVar()
CenZ.set(z0)
manaut=tk.DoubleVar()
manaut.set(1)
carstatus=tk.StringVar()
carstatus.set("at rest")
pumpstatus=tk.StringVar()
pumpstatus.set("OFF")
dx = tk.DoubleVar()
dy = tk.DoubleVar()
root.config(bg="#6FAFE7")
frame1=tk.Frame(root,borderwidth = 2,bg="lavender",height='100',width='1500')
frame1.place(x=0,y=0)
frame2=tk.Frame(root,bg="black",borderwidth=2)
frame2.place(x=30,y=140)
frame3=tk.Frame(root,bg="black",height='400',width='500')
frame3.place(x=30,y=160)
frame4 = tk.Frame(root,borderwidth = 2, bg="snow3",height='140',width='1163')
frame4.place(x=0,y=610)
frame5=tk.Frame(root,bg="black",height='400',width='500')
frame5.place(x=580,y=160)
frame6=tk.Frame(root,bg="black",bd=2)
frame6.place(x=580,y=140)
frame7=tk.Frame(root,borderwidth = 20,bd=20,bg="light gray",height='648',width='335')
frame7.place(x=1165,y=102)
frame8=tk.Frame(frame4,borderwidth = 6,bd=6,bg="snow2",height='130',width='740')
frame8.place(x=420,y=3)
frame9=tk.Frame(frame7,bg="snow2",height='300',width='300')
frame9.place(x=0,y=80)
lmain = tk.Label(frame3)
lmain2=tk.Label(frame5)
global background
def man():
    red['state']=tk.DISABLED;blue['state']=tk.DISABLED;
    All['state']=tk.DISABLED;pick['state']=tk.DISABLED;
    Or['state'] = tk.DISABLED
    Cfb['state']=tk.NORMAL;Cfl['state']=tk.NORMAL;
    Cfw['state'] = tk.NORMAL;Cfr['state'] = tk.NORMAL;
    FKin['state'] = tk.NORMAL;
    BKin['state'] = tk.NORMAL;
    Cfs['state'] = tk.NORMAL;
    Green['state'] = tk.DISABLED
    Cfwc['state'] = tk.NORMAL;Cfbc['state']=tk.NORMAL;
    manaut.set(0);
    status.set("No further Automation \n Manual control enabled")
def selfc():
    red['state'] = tk.NORMAL;
    blue['state'] = tk.NORMAL;
    All['state'] = tk.NORMAL;
    pick['state'] = tk.NORMAL;
    Or['state']=tk.NORMAL;Green["state"]=tk.NORMAL
    Green['state']=tk.NORMAL
    FKin['state'] = tk.DISABLED;
    BKin['state'] = tk.DISABLED;
    manaut.set(1);
    status.set("Automation  enabled")
def nothing():
    return
def cameraconfig():
    global background
    url1.set(str(e1.get()))
    url2.set(str(e2.get()))
    sel['state']=tk.NORMAL;
    none['state']=tk.NORMAL;
    fc['state']=tk.NORMAL
    #background=shape.back(url1.get())
    top.destroy()
def cam():
    global top
    global e1
    global e2
    top = tk.Toplevel(root)
    top.title("camera settings")
    top.config(bg="white smoke")
    e1 = tk.Entry(top, width=55, bd=1)
    e1.delete(0, tk.END)
    e1.insert(0, url1.get())
    url1.set(str(e1.get()))
    e1.place(x=230, y=30)
    e2 = tk.Entry(top, width=55, bd=1)
    e2.delete(0, tk.END)
    e2.insert(0, url2.get())
    url2.set(str(e2.get()))
    e2.place(x=230, y=70)
    cam1ip = tk.Label(top, text="Ip address of the front camera: ", bg="ghost white")
    cam1ip.place(x=30, y=30)
    cam2ip = tk.Label(top, text="Ip address of the Robot camera: ", bg="ghost white")
    cam2ip.place(x=30, y=70)
    Okay1 = tk.Button(top, text="OK", command=cameraconfig)
    Okay1.place(x=500, y=110)
    # specify size
    top.geometry("580x150")

def ards():
    top2 = tk.Toplevel(root)
    top2.title("Arduino settings")
    top2.config(bg="white smoke")
    url1 = "COM7"
    e3 = tk.Entry(top2, width=25, bd=1)
    e3.insert(0, url1)
    e3.place(x=80, y=30)
    port = tk.Label(top2, text="Port: ", bg="ghost white")
    port.place(x=30, y=30)
    myFont = font.Font(size=16)
    Okay3 = tk.Button(top2, text="OK", command=top2.destroy)
    Okay3.place(x=200, y=65)
    st3 = str(e3.get());
    # specify size
    top2.geometry("250x100")
    top2.mainloop()
def show_frame(c):
    global lmain
    if (c == "none"):
        [frame, ctr] = ip2.wcam(url1.get())
        cenx.set(ctr[0]);
        ceny.set(ctr[1]);
        ox = float('nan');oy = float('nan')
        olx.set(ox);
        oly.set(oy)
    else:
        [frame, ctr] = ip2.detect(c,url1.get())
        cenx.set(ctr[0]);
        ceny.set(ctr[1]);

    cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img = Image.fromarray(cv2image)
    imgtk = ImageTk.PhotoImage(image=img)
    lmain.imgtk = imgtk
    lmain.configure(image=imgtk)
    lmain.after(10, lambda: show_frame(c))

def show_frame2(c):
    global lmain2
    [frame0, ctr0] = ipn.wcam(url2.get())
    cv2image0 = cv2.cvtColor(frame0, cv2.COLOR_BGR2RGB)
    img0 = Image.fromarray(cv2image0)
    imgtk0 = ImageTk.PhotoImage(image=img0)
    lmain2.imgtk0 = imgtk0
    lmain2.configure(image=imgtk0)
    lmain2.after(10, lambda: show_frame2(c))

def figshow(fr, c):
    global lmain
    global lmain2
    lmain.destroy()
    lmain2.destroy()
    cl.set(c)
    lmain = tk.Label(fr)
    lmain.grid(row=0, column=0)
    show_frame(c)

def fig2():
    global lmain2
    global lmain
    lmain2.destroy()
    lmain.destroy()
    c="none"
    lmain2 = tk.Label(frame5)
    lmain2.grid(row=0, column=0)
    thr= threading.Thread(target=lambda : show_frame2(c));
    thr.start()

def Cardir(dir):
    if(dir=="FOR"):
        [m1,m2]=ard_py.step_up(1,1);
        carstatus.set("slightly forwarded and now at rest")
    elif(dir=="BACK"):
        [m1,m2]=ard_py.step_up(-1,-1);
        carstatus.set("slightly turns backward and now at rest")
    elif(dir=="LEFT"):
        [m1,m2]=ard_py.step_up(-1,1);
        carstatus.set("slightly turns LEFT and now at rest")
    elif(dir=="RIGHT"):
        [m1,m2]=ard_py.step_up(1,-1);
        carstatus.set("slightly turns RIGHT and now at rest")
    elif (dir == "FORC"):
        carstatus.set("moving FORWARD")
        [m1, m2] = ard_py.step_up(11, 11);
    elif (dir == "BACKC"):
        carstatus.set("moving BACKWARD")
        [m1, m2] = ard_py.step_up(-11, -11);
    elif(dir=="pump_on"):
        Ard_pycmd.step_up(0, 0, 0, 0)
        pumpstatus.set("ON")
    elif(dir=="pump_off"):
        Ard_pycmd.step_up(0, 0, 0, 1)
        pumpstatus.set("OFF")
    elif (dir == "reset"):
        Ard_pycmd.step_up(140, 140,140,1)
        pumpstatus.set("OFF")
        Ard_pycmd.step_up(-140, -140, -140, 1)
    else:
        [m1,m2]=ard_py.step_up(0,0);
        carstatus.set("at rest")
def Automate():
    col=cl.get()
    Z0=CenZ.get()
    k=1;
    for i in range(20):
        al = 100;bl = 50;le = 280;lf = 100;
        if (manaut.get() == 0):
            status.set("Automation fully stopped")
            break;
        [cx,cy]=coordinate.coord(cl.get(),url1.get())
        status.set("Automation happening")
        if (cx==0):
            break;
        [th1p, th2p, th3p] = trajectory.steppath(cx, cy, Z0-51.4, "pick")
        r, = th1p.shape
        for i in range(r):
            stx = th1p[i] * 8.88888;sty = th2p[i] * 8.88888;stz = th3p[i] * 8.88888;
            stx = round(stx / 2);sty = round(sty / 2);stz = round(stz / 2);
            stx = int(stx);sty = int(sty);stz = int(stz);
            [m1, m2, m3, m4] = Ard_pycmd.step_up(stx, sty, stz, 0)
            sv1.set(m1[1][0]);sv2.set(m2[1][0]);sv3.set(m3[1][0]);pumpstatus.set("ON")
        time.sleep(2);
        [th1pl, th2pl, th3pl] = trajectory.steppath(cx, cy, Z0-51.4, "place")
        r, = th1pl.shape
        for i in range(r):
            stx = th1pl[i] * 8.88888;sty = th2pl[i] * 8.88888;stz = th3pl[i] * 8.88888;
            stx = round(stx / 2);sty = round(sty / 2);stz = round(stz / 2);
            stx = int(stx);sty = int(sty);stz = int(stz);
            [m1, m2, m3, m4] = Ard_pycmd.step_up(stx, sty, stz, 0)
            sv1.set(m1[1][0]);sv2.set(m2[1][0]);sv3.set(m3[1][0]);pumpstatus.set("ON")
        Ard_pycmd.step_up(0, 0, 0, 1)
        pumpstatus.set("OFF")
        time.sleep(5);
        [th1r, th2r, th3r] = trajectory.steppath(cx, cy, Z0-51.4, "return")
        r, = th1r.shape
        for i in range(r):
            stx = th1r[i] * 8.88888;sty = th2r[i] * 8.88888;stz = th3r[i] * 8.88888;
            stx = round(stx / 2);sty = round(sty / 2);stz = round(stz / 2);
            stx = int(stx);sty = int(sty);stz = int(stz);
            [m1, m2, m3, m4] = Ard_pycmd.step_up(stx, sty, stz, 1)
            sv1.set(m1[1][0]);sv2.set(m2[1][0]);sv3.set(m3[1][0]);pumpstatus.set("OFF")
        Ard_pycmd.step_up(4, 8, 10, 1)
        time.sleep(3)
def Aut2():
    return None
def thfunc1():
    global thrd

    thrd = threading.Thread(target=Automate)
    # Start the thread
    thrd.daemon = True
    thrd.start()

menubar =tk.Menu(root,borderwidth=5)
pixelVirtual = tk.PhotoImage(width=1, height=1)
none=tk.Button(frame1,text="Start \n camera",state=tk.DISABLED,width=8,height=5,command=lambda: figshow(frame3,"none"))
none.place(x=10,y=5)
red=tk.Button(frame1,text="Red",state=tk.DISABLED,width=5,height=3,command=lambda: figshow(frame3,"r"))
myFont = font.Font(family='Helvetica')
red['font'] = myFont
red.place(x=75,y=5)
blue=tk.Button(frame1,text="Blue",state=tk.DISABLED,width=5,height=3,command=lambda: figshow(frame3,"b"))
blue['font'] = myFont
blue.place(x=144,y=5)
Green=tk.Button(frame1,text="Green",state=tk.DISABLED,width=5,height=3,command=lambda: figshow(frame3,"g"))
Green['font'] = myFont
Green.place(x=212,y=5)
Yl=tk.Button(frame1,text="Yellow",width=5,height=3,command=nothing)
Yl['font'] = myFont
Yl.place(x=282,y=5)
Or=tk.Button(frame1,text="Orange",state=tk.DISABLED,width=5,height=3,command=lambda: figshow(frame3,"O"))
Or['font'] = myFont
Or.place(x=352,y=5)
pink=tk.Button(frame1,text="pink",width=5,height=3,command=nothing)
pink['font'] = myFont
pink.place(x=422,y=5)
All=tk.Button(frame1,text="All \n color",width=8,height=5,command=lambda: figshow(frame3,"All"))
All.place(x=522,y=5)
pick=tk.Button(frame1,text="Automate \n pick the \n object",width=10,height=5,command=thfunc1)
pick.place(x=600,y=5)
fc=tk.Button(frame1,text="front camera",state=tk.DISABLED,width=10,height=5,command=fig2)
fc.place(x=690,y=5)
manual=tk.Button(frame1,text="Manual Control",width=15,height=3,command=man)
manual.place(x=1250,y=5)
manual['font'] = myFont
sel=tk.Button(frame1,text="Automatic Control",state=tk.DISABLED,width=15,height=3,command=selfc)
sel.place(x=1070,y=5)
sel['font'] = myFont
myFont2 = font.Font(size=8)
cam1 = tk.Label(frame2, text='Robot camera',width=81)
cam1['font'] = myFont2
cam1.grid(row=0,column=0,padx=2)
cam2 = tk.Label(frame6, text='front camera',width=81)
cam2['font'] = myFont2
cam2.grid(row=0,column=0,padx=2)
tx=str(cenx)
ty=str(ceny)
stam=str(status)
labx = tk.Label(frame4, text="X coordinate of the object: ",bg="snow3")
labx.place(x=30,y=15)
labx['font'] = myFont
labvx = tk.Label(frame4,bg="snow3", textvariable = tx)
labvx.place(x=280,y=15)
labvx['font'] = myFont
stashx = tk.Label(frame4, textvariable=stam ,bg="snow3")
stashx.place(x=30,y=70)
stashx['font'] = myFont
laby = tk.Label(frame4,bg="snow3", text="Y coordinate of the object: ")
laby.place(x=30,y=40)
laby['font'] = myFont
labvy = tk.Label(frame4,bg="snow3", textvariable = ty)
labvy.place(x=280,y=40)
labvy['font'] = myFont
tmx=str(sv1)
tmy=str(sv2)
tmz=str(sv3)
tp=str(pumpstatus)
tcar=str(carstatus)
mx = tk.Label(frame8, text="motor_1_step: ",bg="snow2")
mx.place(x=30,y=10)
mx['font'] = myFont
mvx = tk.Label(frame8,bg="snow2", textvariable = tmx)
mvx.place(x=168,y=10)
mvx['font'] = myFont
my = tk.Label(frame8,bg="snow2", text="motor_2_step: ")
my.place(x=248,y=10)
my['font'] = myFont
mvy = tk.Label(frame8,bg="snow2", textvariable = tmy)
mvy.place(x=386,y=10)
mvy['font'] = myFont
mz = tk.Label(frame8,bg="snow2", text="motor_3_step: ")
mz.place(x=466,y=10)
mz['font'] = myFont
mvz = tk.Label(frame8,bg="snow2", textvariable = tmz)
mvz.place(x=606,y=10)
mvz['font'] = myFont
pumpis=tk.Label(frame8,bg="snow2",text="pump is: ")
pumpis.place(x=30,y=37)
pumpis['font'] = myFont
pumpv=tk.Label(frame8,bg="snow2",textvariable=tp)
pumpv.place(x=110,y=37)
pumpv['font'] = myFont
Caris=tk.Label(frame8,bg="snow2",text="Car is: ")
Caris.place(x=30,y=65)
Caris['font'] = myFont
Carv=tk.Label(frame8,bg="snow2",textvariable=tcar)
Carv.place(x=95,y=65)
Carv['font'] = myFont
Home=tk.Button(frame7,text="Reset Home",width=10,height=2,command=lambda: Cardir("reset"))
Home.place(x=5,y=10)
Car = tk.Label(frame7,bg="light gray", text="Car steering")
Car.place(x=110,y=550)
Car['font'] = myFont
Cfw=tk.Button(frame7,text="Forward",state=tk.DISABLED,width=10,height=2,command=lambda: Cardir("FOR"))
Cfw.place(x=150,y=400)
Cfb=tk.Button(frame7,text="Backward",state=tk.DISABLED,width=10,height=2,command=lambda: Cardir("BACK"))
Cfb.place(x=150,y=500)
Cfl=tk.Button(frame7,text="Left",width=6,state=tk.DISABLED,height=3,command=lambda: Cardir("LEFT"))
Cfl.place(x=110,y=442)
Cfr=tk.Button(frame7,text="Right",state=tk.DISABLED,width=6,height=3,command=lambda: Cardir("RIGHT"))
Cfr.place(x=218,y=442)
Cfs=tk.Button(frame7,text="stop",state=tk.DISABLED,width=6,height=3,command=lambda: Cardir("stop"))
Cfs.place(x=5,y=512)
Cfwc=tk.Button(frame7,text="Forward \n Continous",state=tk.DISABLED,width=10,height=2,command=lambda: Cardir("FORC"))
Cfwc.place(x=5,y=400)
Cfbc=tk.Button(frame7,text="Backward \n Continous",state=tk.DISABLED,width=10,height=2,command=lambda: Cardir("BACKC"))
Cfbc.place(x=5,y=460)
PUMpon=tk.Button(frame7,text="Pump \n ON",width=10,height=2,command=lambda :Cardir("pump_on"))
PUMpon.place(x=100,y=10)
Pumpoff=tk.Button(frame7,text="Pump \n OFF",width=10,height=2,command=lambda :Cardir("pump_off"))
Pumpoff.place(x=180,y=10)
Kin = tk.Label(frame9,bg="snow2", text="Kinematics")
Kin.place(x=105,y=10)
Kin['font'] = myFont
FKin = tk.Button(frame9,bg="snow2",state=tk.DISABLED, text="Forward \n Kinematics", command =nothing)
FKin.place(x=5,y=90)
BKin = tk.Button(frame9,bg="snow2",state=tk.DISABLED, text="Inverse \n Kinematics", command =nothing)
BKin.place(x=5,y=180)
t1pos=tk.Label(frame9,bg="light gray",width=6,height=2, text="Theta1")
t1pos.place(x=90,y=50)
t2pos=tk.Label(frame9,bg="light gray", width=6,height=2,text="Theta2")
t2pos.place(x=160,y=50)
t3pos=tk.Label(frame9,bg="light gray",width=6,height=2, text="Theta3")
t3pos.place(x=230,y=50)
t1epos=tk.Entry(frame9,state=tk.DISABLED,width=6)
t1epos.place(x=90,y=90)
t2epos=tk.Entry(frame9,state=tk.DISABLED, width=6)
t2epos.place(x=160,y=90)
t3epos=tk.Entry(frame9,state=tk.DISABLED,width=6)
t3epos.place(x=230,y=90)
Xpos=tk.Label(frame9,bg="light gray",width=6,height=2, text="X")
Xpos.place(x=90,y=140)
Ypos=tk.Label(frame9,bg="light gray", width=6,height=2,text="Y")
Ypos.place(x=160,y=140)
Zpos=tk.Label(frame9,bg="light gray",width=6,height=2, text="Z")
Zpos.place(x=230,y=140)
Xepos=tk.Entry(frame9,state=tk.DISABLED,width=6)
Xepos.place(x=90,y=180)
Yepos=tk.Entry(frame9,state=tk.DISABLED, width=6)
Yepos.place(x=160,y=180)
Zepos=tk.Entry(frame9,state=tk.DISABLED,width=6)
Zepos.place(x=230,y=180)
Fok = tk.Button(frame9,bg="snow2", state=tk.DISABLED,text="Okay", command =nothing)
Fok.place(x=170,y=250)
Fplot = tk.Button(frame9,bg="snow2",state=tk.DISABLED, text="3D plot", command =nothing)
Fplot.place(x=220,y=250)
file = tk.Menu(menubar, tearoff = 0)
menubar.add_cascade(label ='Tools', menu = file)
file.add_command(label ='Camera settings', command = cam)
file.add_separator()
file.add_command(label="Arduino setting", command= ards)
file.add_separator()
file.add_command(label ='Exit', command = root.destroy)
help = tk.Menu(menubar, tearoff=0)
help.add_command(label="About")
menubar.add_cascade(label="Help", menu=help)
root.config(menu = menubar)
root.mainloop()
Ard_pycmd.step_up(0,0,0,1)
Cardir("stop");

