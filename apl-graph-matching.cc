/**
   File: apl-graph-matching.cc
   Description: implementation file for APL graph matching
   Date: 5/6/09
   Maker: jaehoon jeong, jjeong@cs.umn.edu
*/

#include <math.h> //hypot()
#include <iostream>
#include <fstream>
#include "allocpool.h"

using namespace std;

#include "apl-graph-matching.h"

Weight::Weight()
{
  weight = 0;
}

Weight::Weight(double weight)
{
  this->weight = weight;
}

WeightDestroyer::WeightDestroyer()
{  //parameterless default constructor

}

void WeightDestroyer::destroy(void *p)
{
  delete (Weight*)p;
}

bool WeightComparator::compatible(void *pa, void *pb)
{ //function for compatibility checking
  Weight *a = (Weight*)pa;
  Weight *b = (Weight*)pb;
  double dist = hypot(a->weight - b->weight, 0);
  //Function hypot is declared in <math.h>
  
  return dist < threshold;
}

/** I/O operators for class Weight */
istream& operator>>(istream& in, Weight &w)
{
  in >> w.weight;
  return in;
}

ostream& operator<<(ostream& out, Weight &w)
{
  out << w.weight;
  return out;
}

/** I/O operators for class Empty */
istream& operator>>(istream& in, Empty &)
{
  //Do nothing!
  return in;
}

ostream& operator<<(ostream& out, Empty &)
{
  //Do nothing!
  return out;
}
