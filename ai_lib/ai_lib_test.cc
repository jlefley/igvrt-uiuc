#include <iostream>
#include <string>
#include <vector>
#include "SparseMatrix.h"

using namespace std;

int main(int argc, char *argv[])
{
	SparseMatrix<int> mat(0);
	
	mat.set(-3,4,1);
	mat.set(2,-1,1);
	mat.set(0,0,1);
	
	cout << mat.get(-3,4) << endl;
	cout << mat.get(2,6) << endl;


	int * submat = new int[81];

	mat.getFullSubMat(submat, -4, -4, 4, 4);

	for(int i = 0; i < 9; i++)
	{
		for(int j = 0; j < 9; j++)
		{
			cout << submat[(i*9)+j] << " ";
		}
		cout << endl;
	}

	delete[] submat;

	vector< SparseMatrix<int>::Element > elems;

	mat.getElementsInRange(elems, -2, -2, 2, 2);

	for(int i = 0; i < elems.size(); i++)
	{
		cout << elems[i].i << " " << elems[i].j << " " << elems[i].val << endl;
	}

	return 0;
}
