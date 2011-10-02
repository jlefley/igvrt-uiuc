/*
 *  SparseMatrix.h
 *  Sparse Matrix
 *
 */

#ifndef SPARSEMATRIX_H
#define SPARSEMATRIX_H

//c++ libs
#include <iostream>
#include <stdexcept>
#include <string>
#include <map>
#include <vector>

using namespace std;


template <class T>
class SparseMatrix
{
public:

	struct Element
	{
		int i;
		int j;
		T val;
	};

	// Constructor. defaultValue is the value to be returned
	// from an unassigned element in the matrix
	SparseMatrix(T defaultValue);
	
	// Get an element in the matrix at row i and col j.
	// If element was not previously set, will return the default value
	// specified in the constructor
	T get(int i, int j);
	
	// Set an element in the matrix at row i and col j.
	void set(int i, int j, T val);
	
	// Un-set all elements in the matrix
	void clear();
	
	// Fills a full matrix mat with the values in the box defined by (i_start,j_start)
	// to (i_end, j_end).  mat must have been reserved prior to calling, and
	// have enough memory to store all the requested elements
	void getFullSubMat(T * mat, int i_start, int j_start, int i_end, int j_end);

	// Get all elements in the box defined by (i_start,j_start) to (i_end,j_end).  Only
	// those elements that were previously set will be returned.  Results will be placed
	// in the elems vector.
	void getElementsInRange(vector<Element> & elems, int i_start, int j_start, int i_end, int j_end);
	
private:
	// private types
	typedef map<int,T> col_t;
	typedef class col_t::iterator col_iter;
	typedef map<int, col_t > mat_t;
	typedef class mat_t::iterator row_iter;
	
	// private data members
	mat_t rows;
	T def;

};

template <class T>
SparseMatrix<T>::SparseMatrix(T defaultValue)
{
	def = defaultValue;
}

template <class T>
T SparseMatrix<T>::get(int i, int j)
{
	row_iter row = rows.find(i);
	
	if(row != rows.end())
	{
		col_iter col = (row->second).find(j);
		
		if(col != (row->second).end())
		{
			return col->second;
		}
	}

	return def;
}

template <class T>
void SparseMatrix<T>::set(int i, int j, T val)
{
	rows[i][j] = val;
}

template <class T>
void SparseMatrix<T>::clear()
{
	rows.clear();
}

template <class T>
void SparseMatrix<T>::getFullSubMat(T * mat, int i_start, int j_start, int i_end, int j_end)
{
	int num_rows = i_end - i_start + 1;
	int num_cols = j_end - j_start + 1;
	int mat_i = 0;
	int mat_j = 0;

	if(num_rows <= 0)
		return;

	if(num_cols <= 0)
		return;

	for(int i = i_start; i <= i_end; i++)
	{
		mat_j = 0;

		for(int j = j_start; j <= j_end; j++)
		{
			mat[(mat_i*num_cols)+mat_j] = get(i,j);

			mat_j++;
		}

		mat_i++;
	}
}

template <class T>
void SparseMatrix<T>::getElementsInRange(vector<Element> & elems, int i_start, int j_start, int i_end, int j_end)
{
	int num_rows = i_end - i_start + 1;
	int num_cols = j_end - j_start + 1;
	int i;
	int j;

	elems.clear();

	if(num_rows <= 0)
		return;

	if(num_cols <= 0)
		return;
	
	for(row_iter rit = rows.lower_bound(i_start); rit != rows.end(); rit++)
	{
		i = rit->first;

		for(col_iter cit = (rit->second).lower_bound(j_start); cit != (rit->second).end(); cit++)
		{
			j = cit->first;

			if(j > j_end)
				break;

			Element e;
			e.i = i;
			e.j = j;
			e.val = cit->second;

			elems.push_back(e);
		}

		if(i > i_end)
			break;
	}
	
}


#endif

