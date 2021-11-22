# Aerial-Manipulator-Workspace
This Matlab code generates a 3D representation of an aerial manipulator workspace for fixed aerial position and aerial rotation of 180 degrees.

The workspace is defined as the total area that the aerial manipulator can reach given a set of angle limits. These limits were defined by the physical design. The angle limits were as follows:

θ1 low=-120 ˚ derived from limits of link 1

θ1 high=-10 ˚ derived from limits of link 1

θ2 low=0 ˚ derived from limits of link 2

θ2 high=140 ˚ derived from limits of link 2

θ2, absolute low = -90 ˚ derived from limits of servo 2

θ2, absolute high = 90*6/7.00 – 90 ˚ derived from limits of servo 2

A loop runs through all possible positions of the manipulator, giving a scatter plot of the workspace. Positions were also limited to x>0 and y<0 to simplify analysis and prevent interference with the drone rotors.

Utilizing a gift-wrapping algorithm in MATLAB, a perimeter of the points was formed. This perimeter was then revolved in 3D and plotted as a mesh.
