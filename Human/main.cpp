#include "Human.h"
#include <igl/boundary_conditions.h>

#include <igl/lbs_matrix.h>
#include <igl/bbw/bbw.h>

#include <igl/viewer/Viewer.h>
#include <algorithm>
#include <igl/jet.h>


//#define CALCULATE_WEIGHT

#ifndef CALCULATE_WEIGHT
#include "ReadWriteEigen.h"
#endif


Human human;
int idx_frame = -1;
int selected = 0;
const Eigen::RowVector3d sea_green(70./255.,252./255.,167./255.);
double anim_t = 1.0;
double anim_t_dir = -0.03;

RotationList pose;

Eigen::MatrixXd U;

bool pre_draw(igl::Viewer & viewer)
{

	return false;
}

void set_color(igl::Viewer &viewer)
{
	Eigen::MatrixXd C;
	igl::jet(human.weight_.col(selected).eval(),true,C);
	viewer.data.set_colors(C);
}

bool key_down(igl::Viewer &viewer, unsigned char key, int mods)
{
	switch(key)
	{
	case ' ':
		viewer.core.is_animating = !viewer.core.is_animating;
		break;
	case '.':
		selected++;
		selected = (std::min)(std::max(selected,0),(int)human.weight_.cols()-1);
		set_color(viewer);
		break;
		break;
	case ',':
		selected--;
		selected = std::min(std::max(selected,0),(int)human.weight_.cols()-1);
		set_color(viewer);
		break;
	}
	return true;
}

void main(){
	human.init();
	human.load_mesh_file("E:\\Human\\Humanmodel\\4.mesh");
	human.load_bvh_file("E:\\Human\\Humanmodel\\4_tpose.bvh");
	human.attached_local_coordinate("E:\\Human\\Human-motion\\01.asf");
	human.attached_motion("E:\\Human\\Human-motion\\01_01.amc");
	Eigen::read_matrix<Eigen::MatrixXd>("weight.txt",human.weight_);
	igl::Viewer viewer;

	viewer.data.set_mesh(human.get_vertices(), human.get_faces());
	viewer.core.trackball_angle.normalize();

	//viewer.data.set_points(human.get_vertices(),sea_green);
	viewer.callback_pre_draw = &pre_draw;
	viewer.callback_key_down = &key_down;
	viewer.core.animation_max_fps = 30.;
	viewer.launch();
}