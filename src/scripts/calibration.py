#!/usr/bin/env python3
import numpy as np

# Create a 3D transformation matrix
x_cam=0.0
y_cam=0.0
transformation_matrix = np.array([
    [1, 0, 0.2462079348509339],
    [0, 1, 0.2638036660380747],
    [0, 0, 1]
])

matrix_4x1 = np.array([[x_cam],   
                       [y_cam],
                       [1]])
result = np.dot(transformation_matrix, matrix_4x1)
print(f"result: {result}")