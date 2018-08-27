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
disp = np.load('15001_disp.npy')
disp_h = disp.shape[0]
disp_w = disp.shape[1]
dispCenter = [disp_h/2.0,disp_w/2.0]

def fivInt(val):
    if val % 5 == 0:
        return val
    else:
        return val+(5-(val % 5))

#window dimensions
win_w = fivInt(round(disp_w*w_ratio))
win_h = round(disp_h*h_ratio)
print(win_w)
print(win_h)

def addList(l1,l2):
    return [x+y for x,y in zip(l1,l2)]

def MakeWindow(center):
    half_w = round(win_w/2.0)
    half_h = round(win_h/2.0)
    TL = addList(center,[-1*half_w,half_h])
    TR = addList(center,[half_w,half_h])
    BL = addList(center,[-1*half_w,-1*half_h])
    BR = addList(center,[half_w,-1*half_h])
    boxCords = [TL,TR,BL,BR]
    return boxCords

class CoarseGrid():
    def __init__(self,BlockCenter):
        self.BlockCenter = BlockCenter
        self.cords = MakeWindow(self.BlockCenter)

x = CoarseGrid([0,0])
print(x.cords)

boxCords = MakeWindow([0,0])
print(boxCords)
