import numpy as np
import cv2


f = np.array([[-3,-10,-3],[0,0,0],[3,10,3]],dtype="double")
i = np.array([[3,5,1,3],[3,4,1,2],[4,3,2,2],[5,2,3,0]],dtype="double")
print(f)
print(i)
print(cv2.filter2D(i,kernel=f,ddepth=-1))
print(cv2.filter2D(i,kernel=cv2.flip(f,-1),ddepth=-1))

