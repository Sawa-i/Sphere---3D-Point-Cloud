#include "CreateSkin.hpp"
#include<iostream>
#include <fstream>
#include <queue> 


using namespace std;

void CreateSkin( const cadcam::mwTPoint3d<double> refPoint, 
				const unsigned long nx, const unsigned long ny, 
				const unsigned long nz, const double sphereRad, 
				mwDiscreteFunction &func, const double deltaT, 
				const double delta, const std::string &skinFileName )
{

	//Your source code here...

	// @ Creating 3D Point Cloud Space  ( Working! )
	vector<vector<vector<int> > > ptCloud(nz,vector<vector<int> >(ny,vector <int> (nx)));
	


	// @ Sphere Move Path and Delete points in Cloud Space ( in Progress! )
	mwDiscreteFunction obj = func;

	int t_start = obj.GetBeginParameter();
	int t_end = obj.GetEndParameter();
	cadcam::mwTPoint3d<double> sphereCenter_start, sphereCenter_end;

	sphereCenter_start = func.Evaluate(t_start);
	sphereCenter_end = func.Evaluate(t_end);


	int c0_x = sphereCenter_start.x();
	int c0_y = sphereCenter_start.y();
	int c0_z = sphereCenter_start.z();

	int c1_x = sphereCenter_end.x();
	int c1_y = sphereCenter_end.y();
	int c1_z = sphereCenter_end.z();


	// DELETE 4 lines
	cout<<"Sphere Start: ("<<c0_x<<","<<c0_y<<","<<c0_z<<")\n";
	cout<<"Sphere End: ("<<c1_x<<","<<c1_y<<","<<c1_z<<")\n";

		// 1) Narror Range of Points to Check whether inside Path (in progress)

	int minX,minY,minZ,maxX,maxY,maxZ;
	if(c0_x<c1_x){
		minX = c0_x;
		maxX = c1_x;
	}
	else{
		minX = c1_x;
		maxX = c0_x;
	}
	minX-=sphereRad;
	maxX+=sphereRad;

	if(c0_y<c1_y){
		minY = c0_y;
		maxY = c1_y;
	}
	else{
		minY = c1_y;
		maxY = c0_y;
	}

	minY-=sphereRad;
	maxY+=sphereRad;
	
	if(c0_z<c1_z){
		minZ = c0_z;
		maxZ = c1_z;
	}
	else{
		minZ = c1_z;
		maxZ = c0_z;
	}

	minZ-=sphereRad;
	maxZ+=sphereRad;

		// - Path of Sphere ( in Progress)
	cadcam::mwTPoint3d<double> pathVector = operator-(sphereCenter_end,sphereCenter_start);

	cadcam::mwTPoint3d<double> calc,calc1, calc2;
	double distance;

	// for(int z=0; z<nz;z++){
	// 	for(int y=0; y<ny;y++){
	// 		for(int x=0; x<nx;x++){
	for(int z=minZ; z<=maxZ;z++){
		for(int y=minY; y<=maxY;y++){
			for(int x=minX; x<=maxX;x++){
				if(x>-1&&x<int(nx)&&y>-1&&y<int(ny)&&z>-1&&z<int(nz)){
					cadcam::mwTPoint3d<double> rPoint(x,y,z);
					
					// For Determining if point lies within cylinder by the path
					
					// d=∥e×(rP−rA)∥∥e∥≤R , Cylinder Check

					calc1 = operator-(rPoint,sphereCenter_start);
					calc2 = operator-(rPoint,sphereCenter_end);

					calc = operator%(pathVector,calc1);
					distance = calc.operator~()/pathVector.operator~();

					if(distance<=sphereRad && operator*(calc1,pathVector)>=0 && operator*(calc2,pathVector)<=0){// Check in Cylinder
						ptCloud[z][y][x] = 1;
						continue;
					}

					// d= ||rP-rA|| <= R
					calc1 = operator-(rPoint,sphereCenter_start);
					distance = calc1.operator~();

					if(distance <= sphereRad){ // Check at Sphere_Start
						ptCloud[z][y][x] = 1;
						continue;
					}

					// d= ||rP-rB|| <= R
					calc1 = operator-(rPoint,sphereCenter_end);
					distance = calc1.operator~();

					if(distance <= sphereRad){ // Check at Sphere_End
						ptCloud[z][y][x] = 1;
					}
				}

			}
		}
	}
	

  // @ Creating and writing top layer in ASCII file ( Working! )
  ofstream MyFile(skinFileName);

  int Z = nz;
  int Y = ny;
  int X = nx;


  int z=Z-1;
  
  queue<vector<int>> queue;

		
	for(int x=0; x<X; x++){
		for(int y=0; y<Y; y++){
			int val = ptCloud[z][y][x];
			if(val!=1){
				MyFile << to_string(x)+" "+to_string(y)+" "+to_string(z)<< endl;
			}
			else{
   				queue.push({x,y});
			}	  
		
		}
	}

	while(!queue.empty()){
		z-=1;
		int qLen = queue.size();
		for(int i=0;i<qLen;i++){
			vector<int> vec = queue.front();
			queue.pop();
			int x =vec[0], y =vec[1];
			if(ptCloud[z][y][x]!=1){
				MyFile << to_string(x)+" "+to_string(y)+" "+to_string(z)<< endl;
			}
			else{
   				queue.push({x,y});
			}
		}
		if(z==0){
			break;
		}
	}


  MyFile.close();





}
