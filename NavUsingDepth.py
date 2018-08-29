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

print(w_ratio)
print(h_ratio)

import numpy as np
import matplotlib.pyplot as plt

disp = np.load('15001_disp.npy')
print(disp)
plt.set_cmap('hot')
plt.imshow(disp)
disp_h = disp.shape[0]
disp_w = disp.shape[1]
dispCenter = [disp_h/2.0,disp_w/2.0]

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
drone_to_object_ratio = 10
micro_win_w = int(win_w/drone_to_object_ratio)
micro_win_h = int(win_h)
grid_size_w = 5
grid_size_h = 9
micro_grid_size_w = grid_size_w*drone_to_object_ratio
micro_grid_size_h = grid_size_h
print(win_w)
print(win_h)

def MakeMicroWinCords(disp_h,disp_w):
    TL = [int(disp_h/2 + grid_size_h*win_h/2) , int(disp_w/2 - grid_size_w*win_w/2)]
    print(TL)
    grid_cords = [[0 for x in range(micro_grid_size_w)] for y in range(micro_grid_size_h)] #cols x rows
    for i in range(micro_grid_size_h):
        for j in range(micro_grid_size_w):
            grid_cords[i][j] = [TL[0]-micro_win_h*i,TL[1]+micro_win_w*j]
    return grid_cords

disp_mean = np.mean(disp)
print(disp_mean)
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

def FindBestPolicy(red_coll_map,row_scan_order,col_scan_order): #find the first non colliding cell
    for i in row_scan_order:
        for j in col_scan_order:
            if red_coll_map[i][j] == 0: #no possible collision
                return i,j

grid_cords = MakeMicroWinCords(disp_h,disp_w)
coll_map = CreateCollisionMap(grid_cords,disp)
red_map = ReduceMap(coll_map)
lshape(coll_map)
lshape(red_map)
row_scan_order = ScanLineGenerator(len(red_map))
col_scan_order = ScanLineGenerator(len(red_map[0]))
best_y,best_x = FindBestPolicy(red_map,row_scan_order,col_scan_order)

print(f'row scan order {row_scan_order}')
print(f'col scan order {col_scan_order}')
print(f'best coord = {best_y},{best_x}')
plt.set_cmap('Greys')
plt.imshow(coll_map)
plt.set_cmap('Greys')
plt.imshow(red_map)
plt.show()
