#input depth image, current direction and vector
#output new direction and velocity vector

#camera parameters
#pixel density
w_cam = 0.00367 #meters
h_cam = 0.00274 #meters
w_cam_px = 2592 #px
h_cam_px = 1944 #px
w_px_density = w_cam_px/w_cam #px in 1m
h_px_density = h_cam_px/h_cam #px in 1m
#focal length
f = 0.0021 #meters

#Drone dimensions
padding = 0.1 #meters
drone_w = 0.45 + padding #meters
drone_h = 0.1 + padding #meters

#Safe Distance
d = 2 #meters

#projection dims
drone_proj_w = f*drone_w/d #meters
drone_proj_h = f*drone_h/d #meters

#ratio to image size
w_ratio = drone_proj_w/w_cam
h_ratio = drone_proj_h/h_cam

#Image dimensions
IMAGE_W = 752 #px
IMAGE_H = 480 #px

print(f'width ratio {w_ratio}')
print(f'height ratio {h_ratio}')

import numpy as np
import matplotlib.pyplot as plt

img_name = '638'
disp_name = img_name+'_disp.npy'
image_name = img_name+'.jpg'
disp = np.load(disp_name)
disp_h = disp.shape[0]
disp_w = disp.shape[1]
dispCenter = [disp_w/2.0,disp_h/2.0]
print(f'disp Center = {dispCenter}')

#image to disparity ratio
I2D_h = IMAGE_H/disp_h
I2D_w = IMAGE_W/disp_w
print(f' I2D: h = {I2D_h}, w = {I2D_w}')

def tenInt(val):
    rem = val%10
    if rem == 0:
        return val
    elif rem < 5:
        return val-rem
    else:
        return val-rem+10

#window dimensions
win_w = tenInt(round(disp_w*w_ratio))
win_h = round(disp_h*h_ratio)
print(win_h)
drone_to_object_ratio = 10
micro_win_w = int(win_w/drone_to_object_ratio)
micro_win_h = int(win_h)
grid_size_w = 5
grid_size_h = 9
micro_grid_size_w = grid_size_w*drone_to_object_ratio
micro_grid_size_h = grid_size_h

def MakeMicroWinCords(disp_h,disp_w):
    TL = [int(disp_h/2 + grid_size_h*win_h/2) , int(disp_w/2 - grid_size_w*win_w/2)] #top left of grid (refernce)
    print(f'grid top left = {TL}')
    grid_cords = [[0 for x in range(micro_grid_size_w)] for y in range(micro_grid_size_h)] #cols x rows
    for i in range(micro_grid_size_h):
        for j in range(micro_grid_size_w):
            grid_cords[i][j] = [TL[0]-micro_win_h*i , TL[1]+micro_win_w*j]
    return grid_cords

disp_mean = np.mean(disp)
print(f'disp_mean {disp_mean}')
DISPARITY_THRESHOLD =  disp_mean #################################
def EvalCell(TL,disp):
    sum = 0
    for i in range(micro_win_h):
        for j in range(micro_win_w):
            sum = sum+disp[TL[0]-i , TL[1]+j] #top to down , left to right
    avg = sum/(micro_win_h*micro_win_w)
    if avg < DISPARITY_THRESHOLD:
        return 0 #mark as safe
    else:
        return 1 #about to collide

def CreateCollisionMap(grid_cords,disp):
    collision_map = [[0 for w in range(micro_grid_size_w)] for h in range(micro_grid_size_h)]
    for i in range(micro_grid_size_h):
        for j in range(micro_grid_size_w):
            collision_map[i][j] = EvalCell(grid_cords[i][j],disp)
    return collision_map

def ReduceMap(collision_map):
    reduced_map = [[0 for w in range(micro_grid_size_w - drone_to_object_ratio + 1)] for h in range(micro_grid_size_h)]
    for i in range(micro_grid_size_h):
        for j in range(micro_grid_size_w - drone_to_object_ratio+1):
            if sum(collision_map[i][j : j + drone_to_object_ratio]) > 0:
                reduced_map[i][j] = 1 #collision predicted
            else:
                reduced_map[i][j] = 0
    return reduced_map

def lshape(lst):
    x = len(lst)
    y = len(lst[0])
    print(f'rows = {x}, columns = {y}')

def ScanLineGenerator(x): #center first border last scan
    x_center = int(x/2)
    x_line = [0 for a in range(x)]
    x_line[0] = x_center
    x_sign = -1
    for i in range(x-1):
        x_line[i+1] = x_line[i] + x_sign*(i+1)
        x_sign = -1*x_sign
    return x_line

def FindBestPolicy(red_coll_map , row_scan_order , col_scan_order): #find the first non colliding cell
    for i in row_scan_order:
        for j in col_scan_order:
            if red_coll_map[i][j] == 0: #no possible collision
                return j,i

def FindCoordinate(x , y , grid_cords):
    TL_coord = grid_cords[y][x] #cordinates of micro window at start of macrowindow
    print(f'Top Left of waypoint = {TL_coord}')
    center_coord = [TL_coord[1]+win_w/2 , TL_coord[0]-win_h/2]
    return center_coord

def CalculateVector(disp_center , disp_waypoint):
    img_waypoint_px = [disp_waypoint[0]*I2D_w , disp_waypoint[1]*I2D_h] #scale to image size in px
    img_waypoint_m = [img_waypoint_px[0]/w_px_density, img_waypoint_px[1]/h_px_density] #convert in meters
    way_point = [img_waypoint_m[0]*d/f , img_waypoint_m[1]*d/f] #convert to real world coordinates at min collision distance
    return way_point,img_waypoint_px

grid_cords = MakeMicroWinCords(disp_h,disp_w)
coll_map = CreateCollisionMap(grid_cords,disp)
red_map = ReduceMap(coll_map)
lshape(coll_map)
lshape(red_map)
row_scan_order = ScanLineGenerator(len(red_map))
col_scan_order = ScanLineGenerator(len(red_map[0]))
best_x,best_y = FindBestPolicy(red_map,row_scan_order,col_scan_order)
disp_way_point = FindCoordinate(best_x,best_y,grid_cords)

print(f'row scan order {row_scan_order}')
print(f'col scan order {col_scan_order}')
print(f'best coord = {best_x},{best_y}')
print(f'way_point {disp_way_point}')

#plotting results

from matplotlib.patches import Arrow
from matplotlib.patches import Circle
import matplotlib.image as mpimg
#from pylab import savefig

fig1,ax1 = plt.subplots(1)
plt.set_cmap('plasma')
ax1.imshow(disp)

fig2,ax2 = plt.subplots(1)
plt.set_cmap('Greys')
ax2.set_xlabel('x')
ax2.set_ylabel('y')
ax2.imshow(coll_map,aspect = 2.5)
fig2.savefig(img_name+'_coll_map.jpg',bbox_inches = 'tight', dpi = 300)

fig3,ax3 = plt.subplots(1)
plt.set_cmap('Greys')
ax3.set_xlabel('x')
ax3.set_ylabel('y')
ax3.imshow(red_map,aspect = 1.72)
fig3.savefig(img_name+'_red_map.jpg',bbox_inches = 'tight', dpi = 300)

if disp_way_point == dispCenter:
    print('Straight')
    fig4, ax4 = plt.subplots(1)
    plt.set_cmap('plasma')
    ax4.imshow(disp)
    patch3 = Circle((dispCenter[0],dispCenter[1]),radius=7,color='red')
    ax4.add_patch(patch3)
    ax4.set_xlabel('x(px)')
    ax4.set_ylabel('y(px)')
    plt.savefig(img_name+'_disp_dir.jpg',bbox_inches = 'tight', dpi = 300)
    plt.show(fig4)

    img = mpimg.imread(image_name)
    fig5, ax5 = plt.subplots(1)
    ax5.imshow(img)
    img_center = [IMAGE_W/2 , IMAGE_H/2]
    patch4 = Circle((img_center[0], img_center[1]),radius=7,color='yellow')
    ax5.add_patch(patch4)
    ax5.set_xlabel('x(px)')
    ax5.set_ylabel('y(px)')
    plt.savefig(img_name+'_img_dir.jpg',bbox_inches = 'tight', dpi = 300)
    plt.show(fig5)

else:
    way_point,img_waypoint_px = CalculateVector(dispCenter,disp_way_point)
    print(f'world way point = {way_point}')
    print(f'img way point = {img_waypoint_px}')

    fig4, ax4 = plt.subplots(1)
    plt.set_cmap('plasma')
    ax4.imshow(disp)
    patch = Arrow(dispCenter[0], dispCenter[1], disp_way_point[0] - dispCenter[0] , disp_way_point[1] - dispCenter[1], width=10, color='red')
    ax4.add_patch(patch)
    ax4.set_xlabel('x(px)')
    ax4.set_ylabel('y(px)')
    plt.savefig(img_name+'_disp_dir.jpg',bbox_inches = 'tight', dpi = 300)
    plt.show(fig4)


    img = mpimg.imread(image_name)
    fig5, ax5 = plt.subplots(1)
    ax5.imshow(img)
    img_center = [IMAGE_W/2 , IMAGE_H/2]
    patch2 = Arrow(img_center[0], img_center[1], img_waypoint_px[0] - img_center[0] , img_waypoint_px[1] - img_center[1], width=20, color='yellow')
    ax5.add_patch(patch2)
    ax5.set_xlabel('x(px)')
    ax5.set_ylabel('y(px)')
    plt.savefig(img_name+'_img_dir.jpg',bbox_inches = 'tight', dpi = 300)
    plt.show(fig5)
