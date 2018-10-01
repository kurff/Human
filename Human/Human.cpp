#include "Human.h"
#include "igl/readOBJ.h"
#include "igl/readMESH.h"
#include "CMU.h"
#include "skeleton.h"
#include "motion.h"

Human::Human(){
	skeleton_ = nullptr;
	motion_ = nullptr;

}

Human::~Human(){



}

bool Human::load_obj_file(string file){
	bool state = igl::readOBJ(file,vertices_,faces_);
	return state;
}

bool Human::load_mesh_file(string file){
	bool state = igl::readMESH(file.c_str(),vertices_,Tet_,faces_);
	return state;
}


void Human::set_mesh(Eigen::MatrixXd V,Eigen::MatrixXi F){
	vertices_ = Eigen::MatrixXd(V.rows(),V.cols());
	for(int i = 0; i < V.rows(); i ++ ){
	  vertices_(i,0) = V(i,0);
	  vertices_(i,1) = V(i,1);
	  vertices_(i,2) = V(i,2);
	}

	faces_ = Eigen::MatrixXi(2*F.rows(),3);
	for(int i = 0; i < F.rows(); i ++ ){
	   faces_(2*i,0) = F(i,0);
	   faces_(2*i,1) = F(i,1);
	   faces_(2*i,2) = F(i,2);
	   faces_(2*i+1,0) = F(i,0);
	   faces_(2*i+1,1) = F(i,2);
	   faces_(2*i+1,2) = F(i,3);
	}

}

void Human::set_matrix_value(Eigen::MatrixXd& matrix, double m[4][4]){
	matrix = Eigen::MatrixXd(4,4);
	for(int i = 0; i < 4; i ++ ){
		for(int j = 0; j < 4; j ++ ){
		   matrix(i,j) = m[i][j];
		}
	}

}

void Human::compute_transform_matrix(BVH::Joint* joint){
	Eigen::MatrixXd X = Eigen::MatrixXd::Zero(4,4);
	
	//X.Zero();
	X(0,0) = 1;
    X(1,1) = cos(joint->rotation[0]);
	X(1,2) = -sin(joint->rotation[0]);
	X(2,1) = sin(joint->rotation[0]);
	X(2,2) = cos(joint->rotation[0]);
	X(3,3) = 1;
	Eigen::MatrixXd Y= Eigen::MatrixXd::Zero(4,4);
	//Y.Zero();
	Y(0,0) = cos(joint->rotation[1]);
	Y(0,2) = sin(joint->rotation[1]);
	Y(1,1) = 1;
	Y(2,0) = - sin(joint->rotation[1]);
	Y(2,2) = cos(joint->rotation[1]);

	Eigen::MatrixXd Z= Eigen::MatrixXd::Zero(4,4);
	//Z.Zero();
	Z(0,0) = cos(joint->rotation[2]);
	Z(0,1) = - sin(joint->rotation[2]);
	Z(1,0) = sin(joint->rotation[2]);
	Z(1,1) = cos(joint->rotation[2]);
	Z(2,2) = 1;
	joint->T_ = X*Y*Z;
}

void Human::compute_transition(BVH::Joint* joint){
	/*joint->p0 = Eigen::MatrixXd(4,1);
	joint->p0(0,0) = joint->init_position[0];
	joint->p0(1,0) = joint->init_position[1];
	joint->p0(2,0) = joint->init_position[2];
	joint->p0(3,0) = 1;*/
}

void Human::forward_kinematic(Posture pose){
	int num_joint = kinematic_.GetNumJoint();
	vector<int> cache;
	int current = 0;
	int parent = 0;
	BVH::Joint* joint = nullptr;

	for(int i = 0; i < num_joint; i ++ ){
        joint = kinematic_.GetJoint(i);
		joint->rotation[0] = 0;
		joint->rotation[1] = 0;
		joint->rotation[2] = 0;
	}

	for(int i = 0; i < skeleton_->num_Bones(); i ++ ){
		 joint = kinematic_.GetJoint(idx_corrs_[i]);
		joint->rotation[0] = pose.bone_rotation[i].x();
		joint->rotation[1] = pose.bone_rotation[i].y();
		joint->rotation[2] = pose.bone_rotation[i].z();
	}



	///// the child of 

	/////////////////////////// first /////////////////////
	cache.clear();
	cache.push_back(0);
	int idx_site = num_joint - 1;

	//BVH::Joint* joint = nullptr;
	Eigen::MatrixXd t;
	while(!cache.empty()){
		current = cache.back();
		joint = kinematic_.GetJoint(current);
		compute_transform_matrix(joint);
		if(current !=0 ){
		   //kinematic_.GetJoint(current)
			BVH::Joint* joint_parent = joint->parent;
			compute_transform_matrix(joint);
		    joint->T_  = joint->T_ * joint_parent->T_;

			t = joint->T_*joint->T0_.inverse()*joint->p0;//* ;
			joint->position[0] = t(3,0);
			joint->position[1] = t(3,1);
			joint->position[2] = t(3,2);
		}

		cache.pop_back();
		for(int i = 0; i < kinematic_.GetJoint(current)->children.size(); i ++ ){
			cache.push_back(kinematic_.GetJoint(current)->children[i]->index);
		}
		if(kinematic_.GetJoint(current)->parent==nullptr) continue;
		if(kinematic_.GetJoint(current)->has_site){
		   idx_site ++;
		   kinematic_.GetJoint(current)->index_site = idx_site;
           
		}




	}






}

void Human::init(){

	cmu_.load_match_name("joint_name_match.txt");



}



void Human::attached_local_coordinate(string asf_file){

	skeleton_ = new Skeleton(asf_file.c_str(),1.0);
	idx_corrs_.clear();
	for(int i = 0; i < skeleton_->num_Bones(); i ++ ){
		string name = skeleton_->idx2name(i);
		map<string,string>::iterator it = cmu_.matches_.find(
			name);
		if(it == cmu_.matches_.end()){continue;}
		BVH::Joint* joint = kinematic_.GetJoint(it->second);
		idx_corrs_.push_back(joint->index);
		set_matrix_value(joint->T0_,skeleton_->get_Bone_idx(i).rot_parent_current);
		cout<<joint->T0_<<endl;
	}
}


void Human::load_bvh_file(string file){
	kinematic_.Load(file.c_str());

	BVH::Joint* joint = nullptr;

	int num_joint = kinematic_.GetNumJoint();
	vector<int> cache;
	int current = 0;
	int parent = 0;
	cache.clear();
	cache.push_back(0);

	while(!cache.empty()){
		current = cache.back();
		joint = kinematic_.GetJoint(current);
		if(current == 0){
			joint->init_position[0] = 0;
			joint->init_position[1] = 0;
			joint->init_position[2] = 0;
		}

		if(current !=0 ){
		   //kinematic_.GetJoint(current)
		   BVH::Joint* joint_parent = joint->parent;
		   joint->init_position[0] = joint->offset[0] + joint_parent->init_position[0];
		   joint->init_position[1] = joint->offset[1] + joint_parent->init_position[1];
		   joint->init_position[2] = joint->offset[2] + joint_parent->init_position[2];
		}

		cache.pop_back();
		for(int i = 0; i < kinematic_.GetJoint(current)->children.size(); i ++ ){
			cache.push_back(kinematic_.GetJoint(current)->children[i]->index);
		}
		if(kinematic_.GetJoint(current)->parent==nullptr) continue;
		if(kinematic_.GetJoint(current)->has_site){
	                
		}
	}

	for(int i = 0; i < num_joint; i ++ ){
	   joint = kinematic_.GetJoint(i);
	   compute_transition(joint);
	}
	

}


void Human::attached_motion(string amc_file){
	motion_ = new Motion(amc_file.c_str(),1.0,skeleton_);
}



