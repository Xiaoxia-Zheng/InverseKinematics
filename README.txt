----------------------
CMSC691 Assignment 2
----------------------
Xiaoxia Zheng / CE83376


----------------------
Command Line Arguments
----------------------
Example: ./main ogre-skeleton.bf ik-b.con output-%05d.pose

	"ogre-skeleton.bf" & "ik-b.con" are the files that we need to input
	"output-%05d.pose" is the format that we output the files. For this
			 project, you'll have one output file.

-----------------
Project sturcture
-----------------
1. Input bone file and ik file from the input.
2. Caculate jacobian value.
3. Caculate (constraints - end effectors)'s value.
4. Use the transpose of jacobian value dot product step 3's value, then we have the theta.
5. Use joints' local coordinates and the theta to update the joints new world coordinates.
6. Use the constraints vector minus end effector's vector and calculate the magenitude to ouput the error.
7. Converge the error.

--------------     
Problems I met
--------------
1. Difficult to figure out how to calculate the jacobian matrix.
2. Index error is very difficult to debug.
