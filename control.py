import math
from roblib import *
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import gridspec

point = [0,0] #point observed
L = 5 #the side of the square has a size 2*L
dt   = 0.025 #time step
X,Y,Th = [],[],[] #lists of x,y and orientation of the robot in the world frame

#plot configurations
fig = plt.figure(figsize=(50,20))
spec = gridspec.GridSpec(ncols=2, nrows=1,width_ratios=[4,2], height_ratios=[1],hspace=0.5)
#plot 2d
ax2 = fig.add_subplot(spec[0])
ax2.set_sketch_params(length = 10)
ax2.xmin=-50
ax2.xmax=50
ax2.ymin=-30
ax2.ymax=30
#plot 3d
ax3 = fig.add_subplot(spec[1], projection='3d')

line = [[],[],[]]
lines = []
lines_in = []
pt_inside = False

#list of points on the contour right and left
cnt_right = []
cnt_left = []


def clear_3d(ax):
    #clear ax 3d and reset configurations
    ax.clear()
    ax.set_xlim3d(-20, 20)
    ax.set_ylim3d(0, 50)
    ax.set_zlim3d(-20, 20)
    ax.set_xlabel("x robot")
    ax.set_ylabel("Time (t)")
    ax.set_zlabel("y robot")
    ax.set_title('Waterfall 3D')

def clear_2d(ax):
    #clear ax 2d and reset configurations
    ax.clear()
    ax.set_xlim(ax.xmin,ax.xmax)
    ax.set_ylim(ax.ymin,ax.ymax)
    ax.set_xlabel("x world")
    ax.set_ylabel("y world")
    ax.set_title('Mosaic 2D')
    if(pt_inside):
        ax.plot(point[0], point[1], 'bo')
    else:
        ax.plot(point[0], point[1], 'ko')


def draw_visible(x,ax):
    #draw visible area considering the robot's position x in the world frame
    x=x.flatten()
    M = array([ [-L,L,L,-L,-L],
                [L,L,-L,-L,L],])
    M=move_motif(M,x[0],x[1],x[2])
    ax.plot(M[0, :], M[1, :], 'k', linewidth = 2)

def draw_contour(ax):
    global cnt_right,cnt_left
    #draw the sensor's contour
    cnt_rl = []
    cnt_lr = []

    cnt_right.append([X[-1] + L*sin(Th[-1]),Y[-1] - L*cos(Th[-1])])
    cnt_left.append([X[-1] - L*sin(Th[-1]),Y[-1] + L*cos(Th[-1])])

    #draw right and left part of the contour
    ax.plot(array(cnt_right)[:,0], array(cnt_right)[:,1], "red", linewidth = 1)
    ax.plot(array(cnt_left)[:,0], array(cnt_left)[:,1], "red", linewidth = 1)

    #draw begin and end of the contour
    M = array([ [-L,L,L,-L,-L],
                [L,L,-L,-L,L],])

    M_begin=move_motif(M,X[-1],Y[-1],Th[-1])
    cnt_rl.append(cnt_right[-1])
    cnt_rl.append([M_begin[0,2],M_begin[1,2]])
    cnt_rl.append([M_begin[0,1],M_begin[1,1]])
    cnt_rl.append(cnt_left[-1])

    M_end=move_motif(M,X[0],Y[0],Th[0])
    cnt_lr.append(cnt_left[0])
    cnt_lr.append([M_end[0,0],M_end[1,0]])
    cnt_lr.append([M_end[0,3],M_end[1,3]])
    cnt_lr.append(cnt_right[0])

    ax.plot(array(cnt_rl)[:,0], array(cnt_rl)[:,1], "red", linewidth = 1)
    ax.plot(array(cnt_lr)[:,0], array(cnt_lr)[:,1], "red", linewidth = 1)

def draw_waterfall(t,ax):
    #draw waterfall 3d
    x = [-L,L,L,-L,-L]
    z = [L,L,-L,-L,L]
    y = [0,0,0,0,0]
    verts = [list(zip(x,y,z))]
    M_first = array([x,y,z])
    ax.plot(M_first[0],M_first[1],M_first[2],color="k")

    y = [t,t,t,t,t]
    M_last = array([x,y,z])
    ax.plot(M_last[0],M_last[1],M_last[2],color="k")

    M_line = array([[M_first[0,0],M_last[0,0]],[0,t],[M_first[2,0],M_last[2,0]]])
    ax.plot(M_line[0],M_line[1],M_line[2],color="k")

    M_line = array([[M_first[0,1],M_last[0,1]],[0,t],[M_first[2,1],M_last[2,1]]])
    ax.plot(M_line[0],M_line[1],M_line[2],color="k")

    M_line = array([[M_first[0,2],M_last[0,2]],[0,t],[M_first[2,2],M_last[2,2]]])
    ax.plot(M_line[0],M_line[1],M_line[2],color="k")

    M_line = array([[M_first[0,3],M_last[0,3]],[0,t],[M_first[2,3],M_last[2,3]]])
    ax.plot(M_line[0],M_line[1],M_line[2],color="k")


def in_rect(x,pnt):
    #returns true if point pnt is in the robot's visible area when it assumes the pose represented by x
    #also return pt1 and pt2, the coordinates of the point in the robot's frame.
    x=x.flatten()
    pt1 = cos(x[2])*(pnt[0]- x[0]) + sin(x[2])*(pnt[1] - x[1])
    pt2 = -sin(x[2])*(pnt[0]- x[0]) + cos(x[2])*(pnt[1] - x[1])
    if((pt1)**2 < L**2 and (pt2)**2 < L**2):
        return True,pt1,pt2
    return False,pt1,pt2

def draw_tank(x,ax,col='darkblue',r=1,w=2):
    #draw dubins car
    x=x.flatten()
    M = r*array([[1,-1,0,0,-1,-1,0,0,-1,1,0,0,3,3,0], [-2,-2,-2,-1,-1,1,1,2,2,2,2,1,0.5,-0.5,-1]])
    M=move_motif(M,x[0],x[1],x[2])
    ax.plot(M[0, :], M[1, :], col, linewidth = w)

def draw(x,t):
    global line,lines,pt_inside,lines_in
    draw_waterfall(t,ax3)
    draw_visible(x,ax2)
    draw_contour(ax2)
    draw_tank(x,ax2)

    #draw point in waterfall and in the mosaic space
    if(len(X) > 0):
        res,pt1,pt2 = in_rect(array([[X[-1],Y[-1],Th[-1]]]).T,point)
        if(len(line[0]) != 0 and pt_inside != res):
            line[1].append(t)
            line[0].append(pt1)
            line[2].append(pt2)
            lines.append(line)
            lines_in.append(pt_inside)
            line = [[],[],[]]
        line[1].append(t)
        line[0].append(pt1)
        line[2].append(pt2)
        pt_inside = res
        if(pt_inside):
            ax3.plot(line[0],line[1],line[2],color="b")
        else:
            ax3.plot(line[0],line[1],line[2],color="k")

        for i in range(len(lines)):
            l = lines[i]
            if(lines_in[i]):
                ax3.plot(l[0],l[1],l[2],color="b")
            else:
                ax3.plot(l[0],l[1],l[2],color="k")

    pause(0.001)

for t in arange(0,10,dt):
    print("t = ",t)
    v = 0.5
    x = array([[(40*cos(t*v)),
                 (20*sin(2*t*v)),
                 arctan2(2*20*v*cos(2*t*v),-2*20*v*sin(t*v)) ]]).T
    X.append(x[0,0])
    Y.append(x[1,0])
    Th.append(x[2,0])
    clear_2d(ax2)
    clear_3d(ax3)
    draw(x,t)

plt.show()
pause(3)
