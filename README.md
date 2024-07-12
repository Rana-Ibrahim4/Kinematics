# Kinematics
## Compute Inverse and forward kinematics with 3 degree of freedom

For the forward kinematics, we need to determine the position and orientation of the end-effector (the last link) given the joint angles of the 3 joints.

The general approach would be:

1. Define the reference frames for each joint using the Denavit-Hartenberg (DH) convention.
2. Compute the transformation matrices between each adjacent frame.
3. Multiply the transformation matrices to get the overall transformation from the base frame to the end-effector frame.

The forward kinematics equations would look something like this:
#### Define DH parameters
ex Git :
```
a1, a2, a3 = link lengths
d1, d2, d3 = link offsets
theta1, theta2, theta3 = joint angles
```
#### Compute transformation matrices
ex Git :
```
T01 = [[cos(theta1), -sin(theta1), 0, a1*cos(theta1)], 
       [sin(theta1),  cos(theta1), 0, a1*sin(theta1)],
       [0, 0, 1, d1],
       [0, 0, 0, 1]]

T12 = [[cos(theta2), -sin(theta2), 0, a2*cos(theta2)],
       [sin(theta2),  cos(theta2), 0, a2*sin(theta2)], 
       [0, 0, 1, d2],
       [0, 0, 0, 1]]

T23 = [[cos(theta3), -sin(theta3), 0, a3*cos(theta3)],
       [sin(theta3),  cos(theta3), 0, a3*sin(theta3)],
       [0, 0, 1, d3], 
       [0, 0, 0, 1]]
```
#### Compute the overall transformation
ex Git :
```
T03 = T01 * T12 * T23
```
#### The position and orientation of the end-effector is stored in T03

For the inverse kinematics, we need to determine the joint angles given the desired position and orientation of the end-effector.

The general approach would be:

1. Derive the inverse kinematics equations using trigonometry and geometric relationships.
2. Solve the equations to find the values of the joint angles.

The inverse kinematics equations would look something like this:
#### Given the desired end-effector position (x, y, z)
ex Git :
```
x, y, z = desired_position
```
#### Solve for the joint angles
ex Git :
```
theta1 = atan2(y, x)
theta2 = atan2(sqrt(x**2 + y**2), z) - atan2(a2, d3)
theta3 = atan2(z - d1 - d2*sin(theta2), a2 + d2*cos(theta2))
```
