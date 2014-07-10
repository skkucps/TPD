/**
   File: apl-graph-matching.h
   Description: header file for APL graph matching
   Date: 5/6/09
   Maker: jaehoon jeong, jjeong@cs.umn.edu
*/

#ifndef __APL_GRAPH_MATCHING_H__
#define __APL_GRAPH_MATCHING_H__

#include <iostream>
#include <fstream>

#include "argraph.h" //for class AttrDestroyer, class AttrComparator

/** class Weight for edge weight */
class Weight
{
  /* Without the following declarations, many errors are issued related to operator >> or << */
  friend istream& operator>>(istream& in, Weight &w);
  friend ostream& operator<<(ostream& out, Weight &w);

 public:

  double weight;

  Weight(); //parameterless default constructor
 
  Weight(double weight); //constructor with parameter
};

/** class Empty */
class Empty
{
  /* Without the following declarations, many errors are issued related to operator >> or << */
  friend istream& operator>>(istream& in, Empty &);
  friend ostream& operator<<(ostream& out, Empty &);
};

/** class Destroyer for Weight */
class WeightDestroyer: public AttrDestroyer
{
 public:
  WeightDestroyer(); //parameterless default constructor

  virtual void destroy(void *p);
};

/** class WeightComparator for weight comparison between two edges */
class WeightComparator: public AttrComparator
{
 private:

  double threshold; //threshold for comparison

 public:

  WeightComparator(double threshold)
  {
    this->threshold = threshold;
  }

  virtual bool compatible(void *pa, void *pb); //function for compatibility checking

};


#endif
