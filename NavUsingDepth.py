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
micro_win_w = int(win_w/10)
micro_win_h = int(win_h)
grid_size_w = 50
grid_size_h = 9
print(win_w)
print(win_h)

def addList(l1,l2):
    return [x+y for x,y in zip(l1,l2)]

def MakeMicroWinCords(disp_h,disp_w):
    TL = [int(disp_h*0.5 + 4.5*win_h) , int(disp_w*0.5 - 2.5*win_w)]
    print(TL)
    grid_cords = [[0 for x in range(grid_size_w)] for y in range(grid_size_h)] #colsxrows
    for i in range(grid_size_h):
        for j in range(grid_size_w):
            grid_cords[i][j] = [TL[0]-micro_win_h*i,TL[1]+micro_win_w*j]
    return grid_cords


scan_order = [4,3,5,2,6,1,7,0,8] #scanning columns

DISPARITY_THRESHOLD = 0.012 #################################
def EvalCell(TL,disp):
    sum = 0
    for i in range(micro_win_h):
        for j in range(micro_win_w):
            sum = sum+disp[TL[0]-i , TL[1]+j] #top to down , left to right
    avg = sum/(micro_win_h*micro_win_w)
    if avg < DISPARITY_THRESHOLD:
        return 1 #mark as safe
    else:
        return 0 #about to collide

def CreateSafeMap(grid_cords,disp):
    safe_map = [[0 for w in range(grid_size_w)] for h in range(grid_size_h)]
    for i in range(grid_size_h):
        for j in range(grid_size_w):
            safe_map[i][j] = EvalCell(grid_cords[i][j],disp)
    return safe_map

grid_cords = MakeMicroWinCords(disp_h,disp_w)
# dispp = np.random.rand(disp_h,disp_w)
mapp = CreateSafeMap(grid_cords,disp)
print(mapp)


plt.set_cmap('hot')
plt.imshow(mapp)
plt.show()
# class Grid():
#     def __init__(self,disp_width,disp_height):
#         self.disp_width = disp_width
#         self.disp_height = disp_height
#         self.grid_cords = MakeMicroWinCords(disp_width,disp_height)
#         self.scan_order = [4,3,5,2,6,1,7,0,8] #scanning columns
#         self.safe_map = [[0 for x in range(grid_size_w)] for y in range(grid_size_h)]
#
#     def MakeSafeMap(self.safe_map,self.scan_order,self.grid_cords,)
# x = Grid(disp_w,disp_h)
# print(x.grid_cords)
