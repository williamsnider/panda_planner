This folder contains contains several edited classes that allow the manipulator's states to be checked that the all points of the manipulator are contained within a sphere. In my experiment, this corresponds to the manipulator not crashing into the shelf structure.

The classes are slightly edited from their MATLAB internal versions. The main changes are in the ManipulatorStateValidatorSphere class, which adds the checking of the points along the collision meshes within the sphere.

Upgrading MATLAB beyond 2022b might cause this to break.