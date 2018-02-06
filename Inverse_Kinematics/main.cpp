//
//  main.cpp
//  project_2
//
//  Created by Xiaoxia Zheng on 9/21/17.
//  Copyright © 2017 Xiaoxia Zheng. All rights reserved.
//

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>


//Read the input bf file.
//Store data to new data structure and return a 2D vector.
std::vector<std::vector<double> > read_bf(std::string bf, int rows, int cols){
	std::vector<std::vector<double> >  bf_data(rows, std::vector<double>(cols, 0));
//	std::fstream bf_file(bf, std::ios_base::in);
	std::fstream bf_file("/Users/zhengli/Desktop/project_1/ogre-skeleton.bf", std::ios_base::in);
	
	if (bf_file.is_open()) {
		for (int i =0; i<rows; i++) {
			for (int j=0; j<cols; j++) {
				bf_file >> bf_data[i][j];
				//printf("%f ", bf_data[i][j]);
			}
		}
	}
	bf_file.close();
	return bf_data;
}


//Read the ik file.
//Store data to new data structure and return a 2D vector.
std::vector< std::vector<double> > read_ik(std::string constr){
//	std::fstream ik_file(constr, std::ios_base::in);
	std::fstream ik_file("/Users/zhengli/Desktop/project_2/ik-b.con", std::ios_base::in);
	int constr_nums;
	int cons_cols = 4;
	ik_file >> constr_nums;
	std::vector< std::vector<double> >  ik_data(constr_nums, std::vector<double>(cons_cols, 0));
	for (int i=0; i<constr_nums; i++) {
		for (int j=0; j<cons_cols; j++) {
			ik_file >> ik_data[i][j];
			//printf("%f ", ik_data[i][j]);
		}
	}
	ik_file.close();
	return ik_data;
}


//Using a recursive function to iterate all joints' parents' transforms.
std::vector<double> locToWorld(std::vector< std::vector<double> > bf, std::vector<double> world, int joint_idx){
	int parent_idx;
	std::vector< std::vector<double> > bf_world;
	
	//If joint's index equals to 0 means the function has iterate to the root.
	//Then the recursive function stops.
	if (joint_idx == 0) {
		return world;
	}else{
		parent_idx = bf[joint_idx][1];
		world[1] += bf[parent_idx][2];
		world[2] += bf[parent_idx][3];
		world[3] += bf[parent_idx][4];
		world = locToWorld(bf, world, parent_idx); //Iterating back to parents'.
		return world;
	}
}

//Calculate [Ei - Pi].
std::vector< std::vector<double> > cal_sub(std::vector< std::vector<double> > bf_world, std::vector< std::vector<double> >  ik){
	std::vector< std::vector<double> > sub_joint;
	for (int i=0; i<ik.size(); i++) {
		std::vector<double> tmp;
		for (int j=0; j<bf_world.size(); j++) {
			for (int k=1; k<4; k++) {
				double temp = bf_world[ ik[i][0] + 1][k] - bf_world[j][k];
				tmp.push_back(temp);
			}
		}
		
		sub_joint.push_back(tmp);
	}
	return sub_joint;
}

//Find out which joint is in the route of end effector to the root.
//Finding these are for calculate the jacobian. Any joints are not in the route suppose to be 0 in the jacobian matrix.
std::vector<int> store_idx(std::vector< std::vector<double> > bf, std::vector<double>  ik_data){
	std::vector<int> route_idx;
	int joint_idx, parent_idx;
	parent_idx = -1;
	joint_idx = ik_data[0];
	while (parent_idx != 0) {
		parent_idx = bf[joint_idx+1][1];
		if (parent_idx == 0) {
			break;
		}else{
			route_idx.push_back(parent_idx);
		}
		joint_idx = parent_idx-1;
	}
	return route_idx;
}


//rows = constrains.size();
//Calculate jacobian values.
//J= [1,0,0]×(Ei-Pi)x  [0,1,0]×(Ei-Pi)x  [0,0,1]×(Ei-Pi)x ]
// 	 [1,0,0]×(Ei-Pi)y  [0,1,0]×(Ei-Pi)y  [0,0,1]×(Ei-Pi)y ]
// 	 [1,0,0]×(Ei-Pi)z  [0,1,0]×(Ei-Pi)z  [0,0,1]×(Ei-Pi)z ]
std::vector< std::vector<double> > calJacobian(std::vector< std::vector<double> > bf_world, std::vector< std::vector<double> > sub, std::vector< std::vector<int> > route_idx, long rows){
	std::vector< std::vector<double> > jacobianVal(rows * 3, std::vector<double>(sub[0].size(), 0));
	//calculate elements in constraints' routes.
	for (int i=0; i<route_idx.size(); i++) {
		for (int j=0; j<route_idx[i].size(); j++) {
			int tmp = route_idx[i][j]; //end effactor joint index.
			int j_colNum = i * 3;
			double x = sub[i][3 * tmp];
			double y = sub[i][3 * tmp + 1];
			double z = sub[i][3 * tmp + 2];
			//(1, 0, 0) X sub(E - P)
			jacobianVal[j_colNum][tmp] = 0;
			jacobianVal[j_colNum + 1][tmp] = -z;
			jacobianVal[j_colNum + 2][tmp] = y;
			//(0, 1, 0) X sub(E - P)
			jacobianVal[j_colNum][(sub[i].size() / 3) + tmp] = z;
			jacobianVal[j_colNum + 1][(sub[i].size() / 3) + tmp] = 0;
			jacobianVal[j_colNum + 2][(sub[i].size() / 3) + tmp] = -x;
			//(0, 0, 1) X sub(E - P)
			jacobianVal[j_colNum][(sub[i].size() * 2 / 3) + tmp] = -y;
			jacobianVal[j_colNum + 1][(sub[i].size() * 2 / 3) + tmp] = x;
			jacobianVal[j_colNum + 2][(sub[i].size() * 2 / 3) + tmp] = 0;
		}
	}
	return jacobianVal;
}


//Calculate desire change to the end effector.
// V = [Gi - Ei]
std::vector< double > calChange(std::vector< std::vector<double> > bf_world, std::vector< std::vector<double> >  ik_data){
	std::vector< double > velocities;
	for (int i=0; i<ik_data.size(); i++) {
		double x = ik_data[i][1] - bf_world[ ik_data[i][0] + 1][1];
		double y = ik_data[i][2] - bf_world[ ik_data[i][0] + 1][2];
		double z = ik_data[i][3] - bf_world[ ik_data[i][0] + 1][3];
		velocities.push_back(x);
		velocities.push_back(y);
		velocities.push_back(z);
	}
	return velocities;
}


//result the theta vector.
// theta = J's transpose . V
std::vector< double > calTheta(std::vector< std::vector<double> > jacobian, std::vector< double >  velocities){
	std::vector< double > theta;
	for (int j=0; j<jacobian[0].size(); j++) {
		double sum = 0;
		
		for (int i=0; i<jacobian.size(); i++) {
			double tmp0 = jacobian[i][j] * velocities[i];
			sum += tmp0;
		}
		theta.push_back(sum);
	}
	return theta;
}


//After getting the theta, we need to translate all joints from the world coordinate back to the local coordinate.
//Because, we need to use the method we used in project 1 to update the new joints' world coordinate.
//Easily using the joints' world coordinate minus the their parents coordinates.
std::vector< std::vector<double> > worldToLoc(std::vector< std::vector<double> > bf, std::vector< std::vector<double> > bf_world){
	std::vector< std::vector<double> > bf_loc(bf.size(), std::vector<double>(4, 0));
	bf_loc[0][0] = bf_world[0][0];
	bf_loc[0][1] = bf_world[0][1];
	bf_loc[0][2] = bf_world[0][2];
	bf_loc[0][3] = bf_world[0][3];
	for (int i=1; i<bf.size(); i++) {
		int parent = bf[bf_world[i][0]+1][1];
		bf_loc[i][0] = bf_world[i][0];
		bf_loc[i][1] = bf_world[i][1] - bf_world[parent][1];
		bf_loc[i][2] = bf_world[i][2] - bf_world[parent][2];
		bf_loc[i][3] = bf_world[i][3] - bf_world[parent][3];
		//		printf("%f %f %f %f\n", bf_loc[i][0], bf_loc[i][1], bf_loc[i][2], bf_loc[i][3]);
	}
	return bf_loc;
}


//Using the same method we did before in project 1 to change the rad to rotation matrix.
std::vector<double> rotationMatrix(float radX, float radY, float radZ){
	float cX = cos(radX);
	float sX = sin(radX);
	
	float cY = cos(radY);
	float sY = sin(radY);
	
	float cZ = cos(radZ);
	float sZ = sin(radZ);
	
	std::vector<double> matrix3f=
	{
		cZ*cY, cZ*sY*sX-sZ*cY, cZ*sY*cX+sZ*sX,
		sZ*cY, sZ*sY*sX+cZ*cX, sZ*sY*cX-cZ*sX,
		-sY, cY*sX, cY*cX
	};
	return matrix3f;
}


//Update the joints' new world coordinate.
std::vector<double> updateToWorld(std::vector< std::vector<double> > bf_init, std::vector<std::vector<double> > bf_loc, std::vector<std::vector<double> > theta, std::vector<double> result, int joint_idx){
	int parent_idx;
	std::vector<double> tmp =
	{
		result[0],
		result[1]*theta[joint_idx][0] + result[2]*theta[joint_idx][1] + result[3]*theta[joint_idx][2],
		result[1]*theta[joint_idx][3] + result[2]*theta[joint_idx][4] + result[3]*theta[joint_idx][5],
		result[1]*theta[joint_idx][6] + result[2]*theta[joint_idx][7] + result[3]*theta[joint_idx][8]
	};
	
	//If joint's index equals to 0 means the function has iterate to the root.
	//Then the recursive function stops.
	if (joint_idx == 0) {
		return tmp;
	}else{
		parent_idx = bf_init[bf_loc[joint_idx][0] + 1 ][1];
		tmp[1] += bf_loc[parent_idx][1];
		tmp[2] += bf_loc[parent_idx][2];
		tmp[3] += bf_loc[parent_idx][3];
		tmp = updateToWorld(bf_init, bf_loc, theta, tmp, parent_idx); //Iterating back to parents'.
		return tmp;
	}
}

//calculate the errors between the contrans' joint coordinate and the joints' new world coordinate.
//Using the magenitude sum to do the error.
double errorUpdate(std::vector<std::vector<double> > newCoord, std::vector< std::vector<double> >  ik_data){
	double error = 0;
	double magenitude;
	for (int i=0; i<ik_data.size(); i++) {
		double joint_idx = ik_data[i][0];
		double x = ik_data[i][1] - newCoord[joint_idx + 1][1];
		double y = ik_data[i][2] - newCoord[joint_idx + 1][2];
		double z = ik_data[i][3] - newCoord[joint_idx + 1][3];
		magenitude = x * x + y * y + z * z;
		error += magenitude;
	}
	return error;
}


int main(int argc, const char * argv[]) {
	std::string ik_input;
	std::vector< std::vector<double> > ikData;
	ik_input = argv[2];
	ikData = read_ik(ik_input); //read constrants file from input.
	
	int bf_cols = 5;
	int bf_rows = 23;
	std::string bf_input;
	bf_input = argv[1];
	std::vector< std::vector<double> > bfData;
	bfData = read_bf(bf_input, bf_rows, bf_cols); // read bone file from input.
	
	//Initiate all joints from local coordinates to world coordinates.
	//for calculate the first jacobian value.
	std::vector< std::vector<double> > bf_world;
	for (int i=0; i<bf_rows; i++) {
		std::vector<double> tmp = {bfData[i][0], bfData[i][2], bfData[i][3], bfData[i][4]};
		std::vector<double> temp = locToWorld(bfData, tmp, i);
		bf_world.push_back(temp);
		//		printf("%f %f %f %f\n", bf_world[i][0], bf_world[i][1], bf_world[i][2], bf_world[i][3]);
	}
	
	double error = 2; //initiate error
	
	//find out all constraints to root's route.
	//store every elements' index in the route.
	std::vector< std::vector<int> > idx;
	for (int i=0; i<ikData.size(); i++) {
		std::vector<int> tmp;
		tmp = store_idx(bfData, ikData[i]);
		idx.push_back(tmp);
	}
	
	int counter = 0;
	//start the iteration.
	//when error < 0.001 && iterations > 10000, means it's converge.
	while (error > 0.0001 && counter < 10000) {
		//first update Ei-Pi
		std::vector< std::vector<double> > sub;
		sub = cal_sub(bf_world, ikData);
		
		//Then calculate jacobian value
		std::vector< std::vector<double> > jacobian;
		jacobian = calJacobian(bf_world, sub, idx, ikData.size());
		
		//find out the velocities
		std::vector< double > velocities;
		velocities = calChange(bf_world, ikData);
		
		//finally get the theta value.
		std::vector< double > theta;
		theta = calTheta(jacobian, velocities);
		
		//change joints world coordinates back to local coordinates.
		std::vector< std::vector<double> > bf_loc;
		bf_loc = worldToLoc(bfData, bf_world);
		
		//	Convert the joints' coordinte to matrix using method rotationMatrix();
		std::vector<std::vector<double> >  coord_matrix;
		long num_joint = theta.size() / 3;
		for (int j=0; j<num_joint; j++) {
			std::vector<double> tmp;
			tmp = rotationMatrix(theta[j], theta[j + num_joint], theta[j + 2 * num_joint]);
			coord_matrix.push_back(tmp);
			//			printf("%f %f %f\n", theta[j], theta[j + theta.size() / ikData.size()], theta[j + 2 * theta.size() / ikData.size()]);
		}
		
		//fihure out the new world coordinates.
		std::vector<std::vector<double> > newCoord;
		for (int i=0; i<bf_loc.size(); i++) {
			std::vector<double> tmp = {bf_loc[i][0], bf_loc[i][1], bf_loc[i][2], bf_loc[i][3]};
			std::vector<double> temp = updateToWorld(bfData, bf_loc, coord_matrix, tmp, i);
			newCoord.push_back(temp);
			//printf("%f %f %f\n", temp[0], temp[1], temp[2]);
		}
		
		//reset the world's coordinates.
		bf_world = newCoord;
		
		//update error.
		error = errorUpdate(newCoord, ikData);
		
		counter++;
		
		printf("step: %d error: %f\n", counter, error);
	}
	
	//Output final coordinates to file
	std::ofstream ouputFile;
	char file_name[200];
	sprintf(file_name, argv[3], 1);
//	sprintf(file_name, "/Users/zhengli/Desktop/project_2/output.pose");
	ouputFile.open(file_name);
	for (int i =0; i<23; i++) {
		for (int j=0; j<4; j++) {
			ouputFile << bf_world[i][j] << ' ';
		}
		ouputFile << '\n';
	}
	ouputFile.close();
}






