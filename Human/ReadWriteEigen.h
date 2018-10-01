#include <Eigen/Geometry>
#include <Eigen/Stdvector>
#include <fstream>

using std::ofstream;
using std::ifstream;

namespace Eigen{
	template<class Matrix>
	void read_matrix(const char* filename, Matrix& matrix){
		ifstream f0;
		int height, width;
		f0.open(filename,ios::in);
		f0 >> height >> width;
		matrix = Matrix(height,width);
		for(int i = 0; i < height; i ++ ){
			for(int j = 0; j < width; j ++ ){
			   f0 >> matrix(i,j);
			}
		}
		f0.close();
	}

	template<class Matrix>
	void save_matrix(const char* filename, Matrix& matrix){
		ofstream f0;
		f0.open(filename,ios::out);
		f0 << matrix.rows() <<" "<< matrix.cols()<<endl;
        f0 << matrix;
		f0.close();
	}

}