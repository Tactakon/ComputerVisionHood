import numpy as np
import cv2 as cv
from scipy.spatial.transform import Rotation
from math import cos, sin, radians
import CalSecondPart
rvecs = CalSecondPart.rvecs
tvecs = CalSecondPart.tvecs

r_obj = Rotation.from_rotvec(np.array(rvecs[0]).reshape(1,3))
rot_matrix = r_obj.as_matrix()

nprvecs = np.array(rvecs)
nptvecs = np.array(tvecs)


xc, xs = cos(radians(nprvecs[0][0][0])), sin(radians(nprvecs[0][0][0]))
yc, ys = cos(radians(nprvecs[0][1][0])), sin(radians(nprvecs[0][1][0]))
zc, zs = cos(radians(nprvecs[0][2][0])), sin(radians(nprvecs[0][2][0]))


tx = nptvecs[0][0][0]
ty = nptvecs[0][1][0]
tz = nptvecs[0][2][0]
translation_mtx = np.array([
    [1,0,0,tx],
    [0,1,0,ty],
    [0,0,1,tz],
    [0,0,0,1]
])


rotation_x_matx = np.array([
    [1,0,0,0],
    [0,xc,-xs,0],
    [0,xs,-xc,0],
    [0,0,0,1]
])

rotation_y_matx = np.array([
    [yc,0,ys,0],
    [0,1,0,0],
    [-ys,0,yc,0],
    [0,0,0,1]
])

rotation_z_matx = np.array([
    [zc,-zs,0,0],
    [zs,zc,0,0],
    [0,0,1,0],
    [0,0,0,1]
])

extrensic_matx = np.dot(rotation_z_matx, np.dot(rotation_y_matx, np.dot(rotation_x_matx, translation_mtx)))


intrinsic_matx = np.append( np.append(CalSecondPart.mtx, [[0],[0],[1]], axis=1), [np.array([0,0,0,1])], axis=0)

print(CalSecondPart.mtx)


camera_matrix = np.dot(intrinsic_matx, extrensic_matx)

print('Intrinsinc matrix: ', intrinsic_matx)

print('Extrinsinc matrix, ', extrensic_matx)

print("Final matrix: ", camera_matrix)


cv.destroyAllWindows()


inverse_mat = np.linalg.inv(camera_matrix)


projection_points = np.array([[2.5],[15],[30],[1]])

real_world_dim = inverse_mat.dot(projection_points)

print(real_world_dim)

print("length along x axis is : ", real_world_dim[0][0])
print("length along y axis is :", real_world_dim[1][0])
print("length along z axis is :", real_world_dim[2][0])