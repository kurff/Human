#pragma once
#include <Eigen/Geometry>
#include <Eigen/Stdvector>
#include <igl/directed_edge_parents.h>
#include <igl/forward_kinematics.h>
#include <igl/deform_skeleton.h>
#include "BVH.h"
#include <string>
using namespace std;
#include "CMU.h"

#include "skeleton.h"
#include "motion.h"


typedef 
  std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond> >
  RotationList;

class Human{
public:
	Human();
	~Human();

public:
	bool load_obj_file(string file);
	void set_mesh(Eigen::MatrixXd V,Eigen::MatrixXi F);
	bool load_mesh_file(string file);
	void load_bvh_file(string file);
	void forward_kinematic(Posture pose);
	
public:
	void attached_motion(string amc_file);
	void attached_local_coordinate(string asf_file);

public:
	inline Eigen::MatrixXi get_faces(){return faces_;}
	inline Eigen::MatrixXd get_vertices(){return vertices_;}


	void init();

	//void save_joint(string file);

private:
	void set_matrix_value(Eigen::MatrixXd& matrix, double m[4][4]);
	void compute_transform_matrix(BVH::Joint* joint);
	void compute_transition(BVH::Joint* joint);

public:
	Eigen::MatrixXd weight_;





private:

	Eigen::MatrixXi faces_;
	Eigen::MatrixXd vertices_;
	Eigen::MatrixXi Tet_;
	BVH kinematic_;
	vector<int> idx_corrs_;

	CMU cmu_;
	Skeleton* skeleton_;
	Motion* motion_;



};