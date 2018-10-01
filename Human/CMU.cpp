#include "CMU.h"
#include <fstream>
using namespace std;

void CMU::load_match_name(string file){
	ifstream f;
	f.open(file,ios::in);
	string joint_name0, joint_name1;
	while(!f.eof()){
	   f >> joint_name0 >> joint_name1;
	   matches_.insert(pair<string, string>(joint_name0,joint_name1));
	}
	f.close();
}

