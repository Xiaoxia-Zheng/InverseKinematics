------------------
Inverse Kinematics
-------------------
For this project, there is a C++ program that will read a bone forest (.bf) file and a constraint (.con) file that contains IK constraints for the skeleton and outputs a series of pose (.pose) files from an iterative IK solver.
There are two parts to this project, (1) computing the Jacobian (2) using the Jacobian to iteratively update joint angles. Assume the first joint is fixed and that all others are 3-dof ball joints (easily treated with Euler angles). A couple things to be careful of: (1) The Jacobian for constraints that are not dependant on the current joint is zero. (2) If you use Euler angles to represent degrees of freedom, be sure to update axes of rotation at every joint.


--------------
Input / Output
--------------
filename		description
ogre-skeleton.bf	.bf "bone forest" file containing the skeleton description. 
			This is a crude file format that stores joint vertex rest positions, corresponding 
			column indices into the weights matrix, and indices of parent joints.
ik.con			"constraint file" containing the constraints. The first line contains the number of 
			constraints. Subsequent lines contain the "weightIndex" followed by the constraint 
			position of the end (outboard joint) of the cooresponding bone.



----------------------
Command Line Arguments
----------------------
Example: ./main ogre-skeleton.bf ik-b.con output-%05d.pose

	"ogre-skeleton.bf" & "ik-b.con" are the files that we need to input
	"output-%05d.pose" is the format that we output the files. For this project, you'll have one output file.


