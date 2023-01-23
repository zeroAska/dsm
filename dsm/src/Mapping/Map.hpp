#pragma once
#include "Utils/EigenTypes.h"

namespace dsm {


  template <typename PointCloud, unsigned int DIM>
  class Map {
  public:
    
    ~Map() = 0;

    virtual void insert_pointcloud(const PointCloud & pc,
                                   const Vector<DIM, float> & center) = 0;
    virtual const decltype(PointCloud().at()) * query(const Vector<DIM, float> & p) = 0;
  }
}
