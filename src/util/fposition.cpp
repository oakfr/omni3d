#include "util/util.h"

bool exist (int q, intVector vector)
{
  for (unsigned int i=0;i<vector.size();i++)
    if (q == vector[i])
      return true;
  return false;
}

void FPositionList::insert(int i, int j, long int pos)
{
  FPosition fp(i,j,pos,true);
  lst.push_back(fp);
}

FPosition FPositionList::get(int i, int j)
{
  FPosition fp;

  for (unsigned int k=0;k<lst.size();k++) {
    fp = lst[k];
    if ((fp.i == i) && (fp.j == j))
      return fp;
  }

  return FPosition(0,0,0,false);
}

void FPositionList::set (int i, int j, long int pos)
{
   FPosition fp;

  for (unsigned int k=0;k<lst.size();k++) {
    fp = lst[k];
    if ((fp.i == i) && (fp.j == j)) {
      lst[k].pos = pos;
      return;
    }
  }

  insert(i,j,pos);
}

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
bool Weight::exist(weightVector vector)
{
	unsigned int i=0;
	for (i=0;i<vector.size();i++) {
		if (vector[i] == *this) return 1;
	}
	return 0;
}

bool Weight::operator== (Weight &w)
{
	if ((i == w.i) && (j == w.j))
		if ((ii == w.ii) && (jj == w.jj))
			return true;
	if ((i == w.ii) && (j == w.jj))
		if ((ii == w.i) && (jj == w.j))
			return true;

	return false;
}
