

#ifndef  _BVH_H_
#define  _BVH_H_


#include <vector>
#include <map>
#include <string>


using namespace  std;
#include <Eigen/Geometry>
#include <Eigen/Stdvector>

class  BVH
{
  public:
	enum  ChannelEnum
	{
		X_ROTATION, Y_ROTATION, Z_ROTATION,
		X_POSITION, Y_POSITION, Z_POSITION
	};
	struct  Joint;

	struct  Channel
	{

		Joint *              joint;

		ChannelEnum          type;

		int                  index;
	};

	struct  Joint
	{

		string               name;
		int                  index;
		Joint *              parent;
		vector< Joint * >    children;
		double               offset[3];
		bool                 has_site;
		double               site[3];
		int                  index_site;
		vector< Channel * >  channels;
		// local coordinate
		double               rot_parent_current[4][4];
		Eigen::MatrixXd T_;
	    Eigen::MatrixXd T0_;

		// rotation
		double rotation[3];

		// the position of current 
		//
		Eigen::MatrixXd p0;
		double init_position[3];

		double position[3];
		double site_position[3];

	};


  private:
	bool                     is_load_success;
	string                   file_name;   
	string                   motion_name; 
	int                      num_channel; 
	vector< Channel * >      channels;    
	vector< Joint * >        joints;      
	map< string, Joint * >   joint_index; 
	int                      num_frame;   
	double                   interval;    
	double *                 motion;


  public:
	BVH();
	BVH( const char * bvh_file_name );
	~BVH();
	void  Clear();
	void  Load( const char * bvh_file_name );

  public:
	bool  IsLoadSuccess() const { return is_load_success; }
	const string &  GetFileName() const { return file_name; }
	const string &  GetMotionName() const { return motion_name; }
	const int       GetNumJoint() const { return  joints.size(); }
	      Joint *   GetJoint( int no ) const { return  joints[no]; }
	const int       GetNumChannel() const { return  channels.size(); }
	const Channel * GetChannel( int no ) const { return  channels[no]; }

	Joint *   GetJoint( const string & j ) const  {
		map< string, Joint * >::const_iterator  i = joint_index.find( j );
		return  ( i != joint_index.end() ) ? (*i).second : NULL; }
    Joint *   GetJoint( const char * j ) const  {
		map< string, Joint * >::const_iterator  i = joint_index.find( j );
		return  ( i != joint_index.end() ) ? (*i).second : NULL; }
	int     GetNumFrame() const { return  num_frame; }
	double  GetInterval() const { return  interval; }
	double  GetMotion( int f, int c ) const { return  motion[ f*num_channel + c ]; }
	void  SetMotion( int f, int c, double v ) { motion[ f*num_channel + c ] = v; }

};



#endif // _BVH_H_
