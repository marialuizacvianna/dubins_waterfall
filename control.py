import math
from roblib import *
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt
from matplotlib import gridspec

point = [0,0]
theta = 0
x = array([[-40,0,0]]).T #x,y,theta
v = 1*0.5
L = 5
dt   = 0.1
X,Y,Th = [],[],[]

# pause(5)
fig = plt.figure(figsize=(50,20))

spec = gridspec.GridSpec(ncols=2, nrows=2,
                         width_ratios=[4,2], height_ratios=[1,2],wspace=0.1, hspace=0.5)

ax2 = fig.add_subplot(spec[0])
ax2.set_sketch_params(length = 10)
ax2.xmin=-50
ax2.xmax=50
ax2.ymin=-30
ax2.ymax=30

ax3 = fig.add_subplot(spec[2], projection='3d')
axq = fig.add_subplot(spec[1])
axq.xmin=-15
axq.xmax=15
axq.ymin=-0.5
axq.ymax=50

line = [[],[],[]]
lines = []
lines_in = []
pt_inside = False
# fig2 = plt.figure(0)
# ax2 = fig2.add_subplot(111, aspect='equal')
# ax2.xmin=-50
# ax2.xmax=50
# ax2.ymin=-15
# ax2.ymax=15
# fig2.add_axes(ax2)

# fig = plt.figure(1)
# ax3 = Axes3D(fig)
# fig.add_axes(ax3)

def clear_3d(ax):
    ax.clear()
    ax.set_xlim3d(-20, 20)
    ax.set_ylim3d(0, 50)
    ax.set_zlim3d(-20, 20)

def clear_2d(ax):
    ax.clear()
    ax.set_xlim(ax.xmin,ax.xmax)
    ax.set_ylim(ax.ymin,ax.ymax)

def f(x,u):
    return array([[x[0,0]+dt*v*cos(x[2,0]),x[1,0]+dt*v*sin(x[2,0]),x[2,0]+dt*u]]).T

def control(delta):
    return -1 + (2*(delta + pi))/(2*pi)

def draw_visible(x,ax):
    x=x.flatten()
    M = array([ [-L,L,L,-L,-L],
                [L,L,-L,-L,L],])
    M=move_motif(M,x[0],x[1],x[2])
    ax.plot(M[0, :], M[1, :], 'k', linewidth = 2)

def draw_contour(ax):
    cnt_right = []
    cnt_left = []
    cnt_rl = []
    cnt_lr = []

    for i in range(len(X)):
        cnt_right.append([X[i] + L*sin(Th[i]),Y[i] - L*cos(Th[i])])
        cnt_left.append([X[i] - L*sin(Th[i]),Y[i] + L*cos(Th[i])])

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

    cnt_right = array(cnt_right)
    cnt_left = array(cnt_left)
    cnt_rl = array(cnt_rl)
    cnt_lr = array(cnt_lr)
    ax.plot(cnt_right[:,0], cnt_right[:,1], "red", linewidth = 1)
    ax.plot(cnt_left[:,0], cnt_left[:,1], "red", linewidth = 1)
    ax.plot(cnt_rl[:,0], cnt_rl[:,1], "red", linewidth = 1)
    ax.plot(cnt_lr[:,0], cnt_lr[:,1], "red", linewidth = 1)

#a fazer
def draw_waterfall(t,ax):
    if(len(X) > 1):
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

        # face_color = [0.5, 0.5, 0.5]
        # collection = Poly3DCollection(verts)
        # collection.set_facecolor(face_color)
        # ax3.add_collection3d(collection)
        # plt.show()
        # plt.show(xx)

def in_rect(x,pnt):
    x=x.flatten()
    pt1 = cos(x[2])*pnt[0] - sin(x[2])*pnt[1] - x[0]
    pt2 = sin(x[2])*pnt[0] + cos(x[2])*pnt[1] - x[1]
    if((pt1)**2 < L**2 and (pt2)**2 < L**2):
        return True,pt1,pt2
    return False,pt1,pt2

def draw_tank(x,ax,col='darkblue',r=1,w=2):
    x=x.flatten()
    M = r*array([[1,-1,0,0,-1,-1,0,0,-1,1,0,0,3,3,0], [-2,-2,-2,-1,-1,1,1,2,2,2,2,1,0.5,-0.5,-1]])
    M=move_motif(M,x[0],x[1],x[2])
    ax.plot(M[0, :], M[1, :], col, linewidth = w)

def draw_quotient(x,t,ax):
    x=x.flatten()
    M = array([[-2*L,-2*L,2*L,2*L,-2*L], [0,2*L + t,2*L + t,0,0]])
    ax.plot(M[0],M[1],color="k")

def draw(x,t):
    global line,lines,pt_inside,lines_in
    draw_waterfall(t,ax3)
    draw_visible(x,ax2)
    draw_contour(ax2)
    draw_tank(x,ax2)
    draw_quotient(x,t,axq)

    if(len(X) > 0):
        res,pt1,pt2 = in_rect(array([[X[-1],Y[-1],Th[-1]]]).T,point)
        if(len(line[0]) != 0 and pt_inside != res):
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
    #     if(res ):
    #         ax3.plot(pt1,i*dt,pt2,color="b")
    #         if(len(x_line) == 0):
    #             if(i == 0):
    #                 axq.plot(-pt2, pt1 + L, 'bo')
    #             else:
    #                 axq.plot(-pt2, 2*L + dt*i, 'bo')
    #
    #         y_line.append(i*dt)
    #         x_line.append(pt1)
    #         z_line.append(pt2)
    # if(len(x_line) > 0):
    #     ax3.plot(x_line,y_line,z_line,color="b")
    # res_before = res

    pause(0.001)

for t in arange(0,10,dt):
    x = array([[(40*cos(t*v)),
                 (20*sin(2*t*v)),
                 arctan2(2*20*v*cos(2*t*v),-2*20*v*sin(t*v)) ]]).T
    X.append(x[0,0])
    Y.append(x[1,0])
    Th.append(x[2,0])
    clear_2d(ax2)
    ax2.set_xlabel("x world")
    ax2.set_ylabel("y world")
    ax2.set_title('Mosaic 2D')
    clear_2d(axq)
    ax2.plot(point[0], point[1], 'bo')
    clear_3d(ax3)
    ax3.set_xlabel("x robot")
    ax3.set_ylabel("Time (t)")
    ax3.set_zlabel("y robot")
    ax3.set_title('Waterfall 3D')
    # err = theta - x[2,0]
    # delta = pi*sin(err/2.)
    # u = control(delta)
    # x = f(x,u)
    # print("delta = ",delta)
    # print("u = ",u)
    print("t = ",t)
    # print("x = ", x)
    # x = array([[(-9.6* t**2  + 11.4*(0.5* t**3 - t) +30),
    #              (-9.6*(0.5* t**3 - t) - 11.4* t**2 + 30),
    #              arctan2((-14.4* t**2 +9.6 -22.8* t),(-19.2* t + 17.1* t**2 - 11.4)) ]]).T
    draw(x,t)

plt.show()
pause(3)
