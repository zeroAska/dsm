#include "VoxelMap.h"
#include "Frame.h"
#include <random>
// updateCovis Debug use
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
// #include <pcl/io/pcd_io.h>

namespace dsm {
  template <typename PointType>
  VoxelMap<PointType>::VoxelMap(float voxelSize) : voxelSize_(voxelSize) {
    // std::cout << "Assigned voxelSize_" << voxelSize << std::endl;
  }

  template <typename PointType>
  bool VoxelMap<PointType>::insert_point(PointType* pt) {
    // 1. find coresponding voxel coordinates
    VoxelCoord intCoord = point_to_voxel_center(pt);
    //std::cout<<"insert the point to "<<intCoord.xc<<", "<<intCoord.yc<<", "<<intCoord.zc<<"\n";    
    // 2. insert point to map
    if (vmap_.count(intCoord)) {
      // voxel already exists
      // std::cout << "Existing voxel" << std::endl;
      std::vector<PointType*>& voxPts = vmap_[intCoord].voxPoints;
      // Check if the point already exists
      for (auto it = voxPts.begin(); it != voxPts.end(); it++) {
        // TODO: make sure this equal is correct
        if (*it == pt) 
          return false; 
      }
      // add only if point didn't exist
      vmap_[intCoord].voxPoints.push_back(pt);
    } else {
      // voxel didn't exist, create voxel and add the point
      vmap_[intCoord] = Voxel<PointType>(intCoord.xc, intCoord.yc, intCoord.zc);
      vmap_[intCoord].voxPoints.push_back(pt);
    }
    // std::cout << "=============================================\n";
    return true;
  }

  template <typename PointType>
  bool VoxelMap<PointType>::delete_point(PointType* pt) {
    // 1. convert to integer coord to look up its voxel
    VoxelCoord intCoord = point_to_voxel_center(pt);
    if (!vmap_.count(intCoord))
      return false;
    // 2. remove this point from the voxel
    std::vector<PointType*>& curVoxPts = vmap_[intCoord].voxPoints;
    // iterate through to find the point to remove
    for (auto it = curVoxPts.begin(); it != curVoxPts.end(); it++) {
      if (*it == pt) {
        curVoxPts.erase(it);
        break;
      }
    }
    // if the voxel contains no point after removal, erase the voxel too
    if (curVoxPts.empty()) {
      vmap_.erase(intCoord);
    }
    return true;
  }

  template <typename PointType>
  bool VoxelMap<PointType>::delete_point_BA(PointType* pt, const Voxel<PointType>* voxel) {
    VoxelCoord intCoord{voxel->xc, voxel->yc, voxel->zc};
    if (!vmap_.count(intCoord))
      return false;
    // 2. remove this point from the voxel
    std::vector<PointType*>& curVoxPts = vmap_[intCoord].voxPoints;
    // iterate through to find the point to remove
    for (auto it = curVoxPts.begin(); it != curVoxPts.end(); it++) {
      if (*it == pt) {
        curVoxPts.erase(it);
        break;
      }
    }
    // if the voxel contains no point after removal, erase the voxel too
    if (curVoxPts.empty()) {
      vmap_.erase(intCoord);
    }
    return true;
  }

  template <typename PointType>
  VoxelMap<PointType>::~VoxelMap() {
    std::cout<<"Voxel map destructed\n";
  }

  template <typename PointType>
  const Voxel<PointType>* VoxelMap<PointType>::query_point(const PointType* pt) const {
    // 1. convert to integer coord to look up its voxel
    VoxelCoord intCoord = point_to_voxel_center(pt);
    if (!vmap_.count(intCoord))
      return nullptr;
    // std::cout << vmap_[intCoord].xc << ", " << vmap_[intCoord].yc << ", " << vmap_[intCoord].zc << std::endl;
    return &vmap_.at(intCoord);
  }

  template <typename PointType>
  const Voxel<PointType>* VoxelMap<PointType>::query_point(float globalX, float globalY, float globalZ) const {
    VoxelCoord intCoord = point_to_voxel_center(globalX, globalY, globalZ);
    //std::cout << "query_point intCoord is "<<intCoord.xc << ", " << intCoord.yc << ", " << intCoord.zc << std::endl;    
    if (!vmap_.count(intCoord))
      return nullptr;

    return &vmap_.at(intCoord);
    
  }  

  template <typename PointType>
  std::unordered_set<int> VoxelMap<PointType>::voxel_seen_frames(PointType* pt) const {
    std::unordered_set<int> resSet;
    const Voxel<PointType>* curVoxel = query_point(pt);
    if (curVoxel == nullptr)
      return resSet;
    for (const PointType* p : curVoxel->voxPoints) {
      resSet.insert(p->currentID());
    }
    return resSet;
  }

  template <typename PointType>
  size_t VoxelMap<PointType>::size() {
    return vmap_.size();
  }

  template <typename PointType>
  const std::vector<PointType*> VoxelMap<PointType>::sample_points() const {
    std::vector<PointType*> res;
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()

    int total_voxels = vmap_.size();
    
    for (const auto& voxelPair : vmap_) {
      const std::vector<PointType*> voxPts = voxelPair.second.voxPoints;
      
      if (voxPts.empty()) {
        std::cout << "Shouldn't be empty, skipping\n";
        //continue;
      }

      else if (voxPts.size() == 1)
        res.push_back(voxPts[0]);
      else {
        std::uniform_int_distribution<int> uniform_dist(0, voxPts.size()-1);
        res.push_back(voxPts[uniform_dist(gen)]);
      }
    }
    return res;
  }

  template <>
  VoxelCoord VoxelMap<SimplePoint>::point_to_voxel_center(const SimplePoint* pt) const {
    //std::cout<<"point world pos is "<<p_wld.transpose();
    // 2. find its corresponding voxel
    std::vector<float> res(3);
    std::vector<float> orig = {pt->x, pt->y, pt->z};
    for (int i = 0; i < 3; i++) {
      // find remainder
      //float rem = fmod(orig[i], voxelSize_);
      //float rem = std::remainder(orig[i], voxelSize_);      
      // float rem = std::remander(orig[i], voxelSize_);
      //if (std::abs(std::abs(rem) - voxelSize_) < 1e-6 ) rem = 0;      
      //int addOne = 0;
      //if (rem >= 0.0)
      //  addOne = rem > (voxelSize_ / 2.0f);
      //else 
      //   addOne = - (std::abs(rem) > (voxelSize_/2.0)); // -(rem < (voxelSize_ / 2.0f));
      //std::cout<<", rem is "<<rem<<", ";
      //res[i] = (int(orig[i] / voxelSize_) + addOne) * voxelSize_;
      res[i] = float(std::lrint(orig[i] / voxelSize_) ) * voxelSize_;
    }
    VoxelCoord resCoord{res[0], res[1], res[2]};
    return resCoord;
  }

  template <>
  VoxelCoord VoxelMap<ActivePoint>::point_to_voxel_center(const ActivePoint* pt) const {
    // 1. get pt coord in world frame
    Eigen::Matrix4f Tcw = pt->reference()->camToWorld().matrix(); //camToWorld
    Eigen::Vector3f p_cam = pt->xyz(); // pt in cam frame
    Eigen::Vector4f p_cam_4;
    p_cam_4 << p_cam(0), p_cam(1), p_cam(2), 1.0;
    Eigen::Vector4f p_wld = Tcw * p_cam_4;
    //std::cout<<"point world pos is "<<p_wld.transpose();
    // 2. find its corresponding voxel
    std::vector<float> res(3);
    std::vector<float> orig = {p_wld(0), p_wld(1), p_wld(2)};
    for (int i = 0; i < 3; i++) {
      // find remainder
      //float rem = fmod(orig[i], voxelSize_);
      //float rem = std::remainder(orig[i], voxelSize_);      
      // float rem = std::remander(orig[i], voxelSize_);
      //if (std::abs(std::abs(rem) - voxelSize_) < 1e-6 ) rem = 0;      
      //int addOne = 0;
      //if (rem >= 0.0)
      //  addOne = rem > (voxelSize_ / 2.0f);
      //else 
      //   addOne = - (std::abs(rem) > (voxelSize_/2.0)); // -(rem < (voxelSize_ / 2.0f));
      //std::cout<<", rem is "<<rem<<", ";
      //res[i] = (int(orig[i] / voxelSize_) + addOne) * voxelSize_;
      res[i] = float(std::lrint(orig[i] / voxelSize_) ) * voxelSize_;
    }
    VoxelCoord resCoord{res[0], res[1], res[2]};
    return resCoord;
  }

  template <typename PointType>
  VoxelCoord VoxelMap<PointType>::point_to_voxel_center(float globalX, float globalY, float globalZ) const {
    // 1. get pt coord in world frame
    Eigen::Vector4f p_wld;
    p_wld << globalX, globalY, globalZ, 1.0;
    //std::cout<<"point world pos is "<<p_wld.transpose();    
    // 2. find its corresponding voxel
    std::vector<float> res(3);
    std::vector<float> orig = {p_wld(0), p_wld(1), p_wld(2)};
    for (int i = 0; i < 3; i++) {
      // find remainder
      //float rem = fmod(orig[i], voxelSize_);
      //float rem = std::remainder(orig[i], voxelSize_);
      //if (std::abs(std::abs(rem) - voxelSize_) < 1e-6 ) rem = 0;
      //float rem = std::remainder()
      //int addOne = 0;
      //if (rem >= 0.0)
      //  addOne = rem > (voxelSize_ / 2.0f);
      //else 
      //  addOne = -(std::fabs(rem) > (voxelSize_ / 2.0f));
      //std::cout<<", rem is "<<rem<<", ";      
      //res[i] = (int(orig[i] / voxelSize_) + addOne) * voxelSize_;
      res[i] = (float)(std::lrint(orig[i] / voxelSize_)) * voxelSize_;
    }
    VoxelCoord resCoord{res[0], res[1], res[2]};
    return resCoord;
  }
  

  template <>
  const Voxel<ActivePoint>* VoxelMap<ActivePoint>::query_point_raycasting(const ActivePoint * pt, float minDist, float maxDist) {
    
    Eigen::Matrix4f Tcw = pt->reference()->camToWorld().matrix(); //camToWorld
    Eigen::Vector3f xyz_cam = pt->xyz();// = //Tcw.block<3,3>(0,0) * pt->xyz() + Tcw.block<3,1>(0,3);
    Eigen::Vector3f xyz_w = Tcw.block<3,3>(0,0) * pt->xyz() + Tcw.block<3,1>(0,3);
    Eigen::Vector3f p_c_normalized = xyz_cam * pt->iDepth();
    Eigen::Vector3f p_cam = p_c_normalized * minDist; // pt in cam frame normalized
    Eigen::Vector3f p_cam_max = p_c_normalized * maxDist;
    Eigen::Vector4f p_cam_4;
    p_cam_4 << p_cam(0), p_cam(1), p_cam(2), 1.0;
    Eigen::Vector3f p = (Tcw * p_cam_4).head<3>();
    Eigen::Vector3f p_max = Tcw.block<3,3>(0,0) * p_cam_max + Tcw.block<3,1>(0,3);

    
    Eigen::Vector3f dir = (p_max - p).normalized();  //(Tcw.block<3,3>(0,0) * (p_max - p_cam)).normalized(); //''Eigen::Map<Eigen::Vector3f>(dirArr);
    
    //int ix = std::floor(p(0));// | 0
    //int iy = std::floor(p(1));// | 0
    //int iz = std::floor(p(2));// | 0
    VoxelCoord p_voxel = point_to_voxel_center(p(0), p(1), p(2));
    float ix = p_voxel.xc;
    float iy = p_voxel.yc;
    float iz = p_voxel.zc;
      
 
    float stepx = ((dir(0) > 0) ? voxelSize_ : -voxelSize_);
    float stepy = ((dir(1) > 0) ? voxelSize_ : -voxelSize_);
    float stepz = ((dir(2) > 0) ? voxelSize_ : -voxelSize_);
    
    float txDelta = std::abs(voxelSize_ / dir(0));
    float tyDelta = std::abs(voxelSize_ / dir(1));
    float tzDelta = std::abs(voxelSize_ / dir(2));
    
    float xdist = (stepx > 0) ? (ix + voxelSize_ /2 - p(0)) : ( ix - voxelSize_/2 - p(0) );
    float ydist = (stepy > 0) ? (iy + voxelSize_ /2 - p(1)) : ( iy - voxelSize_/2 - p(1));
    float zdist = (stepz > 0) ? (iz + voxelSize_ /2 - p(2)) : ( iz - voxelSize_/2 - p(2));    

    //float txMax = (txDelta < std::numeric_limits<float>::max()) ? txDelta * xdist : std::numeric_limits<float>::max();
    //float tyMax = (tyDelta < std::numeric_limits<float>::max()) ? tyDelta * ydist : std::numeric_limits<float>::max();
    //float tzMax = (tzDelta <  std::numeric_limits<float>::max()) ? tzDelta * zdist :  std::numeric_limits<float>::max();
    float txMax = (txDelta < std::numeric_limits<float>::max()) ? xdist / dir(0) : std::numeric_limits<float>::max();
    float tyMax = (tyDelta < std::numeric_limits<float>::max()) ? ydist / dir(1) : std::numeric_limits<float>::max();
    float tzMax = (tzDelta <  std::numeric_limits<float>::max()) ? zdist / dir(2) :  std::numeric_limits<float>::max();
        
    int steppedIndex = -1 ;

    float t = 0;
    VoxelCoord p_voxel_actual = point_to_voxel_center(xyz_w(0), xyz_w(1), xyz_w(2));
    //std::cout<<"voxelSize is "<<voxelSize_<<"Starting position "<<p.transpose()<<", ending position "<<p_max.transpose()<<", actual depth "<<xyz_w.transpose()
    //         <<", actual voxel coord is "<<p_voxel_actual.xc<<", "<<p_voxel_actual.yc<<", "<<p_voxel_actual.zc<<"\n";
    while (t < maxDist) {
      //Eigen::Vector3f curr_p = p + dir * t;
      //p_voxel_actual = point_to_voxel_center(curr_p(0), curr_p(1), curr_p(2));
      /// std::cout<<"Current Point "<<curr_p.transpose()<< ", query voxel "<<ix<<", "<<iy<<", "<<iz<<", t="<<t<<", raw position is "<<xyz_w.transpose()
      //         <<", actual voxel coord is "<<p_voxel_actual.xc<<", "<<p_voxel_actual.yc<<", "<<p_voxel_actual.zc
      //         <<std::endl;              
      const Voxel<ActivePoint> * nextVoxel = query_point( ix, iy, iz);
      
      if (nextVoxel) {
        return nextVoxel;
      }
      if (txMax < tyMax) {
        if (txMax < tzMax) {
          ix += stepx;
          t = txMax;
          txMax += txDelta;
          // tyMax += txDelta * dir(1);
          //tzMax += txDelta * dir(2);
          //std::cout<<"Update tx, tx="<<txMax<<", ty="<<tyMax<<", tzMax="<<tzMax<<std::endl;
          //steppedIndex = 0;
        } else {
          iz += stepz;
          t = tzMax;
          tzMax += tzDelta;
          //std::cout<<"Update tz, tx="<<txMax<<", ty="<<tyMax<<", tzMax="<<tzMax<<std::endl;
          //steppedIndex = 2;
        }
      } else {
        if (tyMax < tzMax) {
          iy += stepy;
          t = tyMax;
          tyMax += tyDelta;
          //std::cout<<"Update ty, tx="<<txMax<<", ty="<<tyMax<<", tzMax="<<tzMax<<std::endl;          
          //steppedIndex = 1;
        } else {
          iz += stepz;
          t = tzMax;
          tzMax += tzDelta;
          // std::cout<<"Update tz, tx="<<txMax<<", ty="<<tyMax<<", tzMax="<<tzMax<<std::endl;          
          //steppedIndex = 2;
        }
      }


    }

    return nullptr;

    // raycast algo paper
    // http://www.cse.chalmers.se/edu/year/2010/course/TDA361/grid.pdf    
    // consider raycast vector to be parametrized by t
    //   vec = [px,py,pz] + t * [dx,dy,dz]

    
  }


  template <typename PointType>
  const Voxel<PointType>* VoxelMap<PointType>::query_point_raycasting(const Eigen::Vector3f & pt_xyz,
                                                                      const Sophus::SE3f & Tcw_sophus,
                                                                      float minDist, float maxDist) {
    
    //Eigen::Matrix4f Tcw = pt->reference()->camToWorld().matrix(); //camToWorld
    Eigen::Matrix4f Tcw = Tcw_sophus.matrix();
    Eigen::Vector3f xyz_cam = pt_xyz;// = //Tcw.block<3,3>(0,0) * pt->xyz() + Tcw.block<3,1>(0,3);
    Eigen::Vector3f xyz_w = Tcw.block<3,3>(0,0) * pt->xyz() + Tcw.block<3,1>(0,3);
    Eigen::Vector3f p_c_normalized = xyz_cam / pt_xyz(2);
    Eigen::Vector3f p_cam = p_c_normalized * minDist; // pt in cam frame normalized
    Eigen::Vector3f p_cam_max = p_c_normalized * maxDist;
    Eigen::Vector4f p_cam_4;
    p_cam_4 << p_cam(0), p_cam(1), p_cam(2), 1.0;
    Eigen::Vector3f p = (Tcw * p_cam_4).head<3>();
    Eigen::Vector3f p_max = Tcw.block<3,3>(0,0) * p_cam_max + Tcw.block<3,1>(0,3);

    
    Eigen::Vector3f dir = (p_max - p).normalized();  //(Tcw.block<3,3>(0,0) * (p_max - p_cam)).normalized(); //''Eigen::Map<Eigen::Vector3f>(dirArr);
    
    //int ix = std::floor(p(0));// | 0
    //int iy = std::floor(p(1));// | 0
    //int iz = std::floor(p(2));// | 0
    VoxelCoord p_voxel = point_to_voxel_center(p(0), p(1), p(2));
    float ix = p_voxel.xc;
    float iy = p_voxel.yc;
    float iz = p_voxel.zc;
      
 
    float stepx = ((dir(0) > 0) ? voxelSize_ : -voxelSize_);
    float stepy = ((dir(1) > 0) ? voxelSize_ : -voxelSize_);
    float stepz = ((dir(2) > 0) ? voxelSize_ : -voxelSize_);
    
    float txDelta = std::abs(voxelSize_ / dir(0));
    float tyDelta = std::abs(voxelSize_ / dir(1));
    float tzDelta = std::abs(voxelSize_ / dir(2));
    
    float xdist = (stepx > 0) ? (ix + voxelSize_ /2 - p(0)) : ( ix - voxelSize_/2 - p(0) );
    float ydist = (stepy > 0) ? (iy + voxelSize_ /2 - p(1)) : ( iy - voxelSize_/2 - p(1));
    float zdist = (stepz > 0) ? (iz + voxelSize_ /2 - p(2)) : ( iz - voxelSize_/2 - p(2));    

    //float txMax = (txDelta < std::numeric_limits<float>::max()) ? txDelta * xdist : std::numeric_limits<float>::max();
    //float tyMax = (tyDelta < std::numeric_limits<float>::max()) ? tyDelta * ydist : std::numeric_limits<float>::max();
    //float tzMax = (tzDelta <  std::numeric_limits<float>::max()) ? tzDelta * zdist :  std::numeric_limits<float>::max();
    float txMax = (txDelta < std::numeric_limits<float>::max()) ? xdist / dir(0) : std::numeric_limits<float>::max();
    float tyMax = (tyDelta < std::numeric_limits<float>::max()) ? ydist / dir(1) : std::numeric_limits<float>::max();
    float tzMax = (tzDelta <  std::numeric_limits<float>::max()) ? zdist / dir(2) :  std::numeric_limits<float>::max();
        
    int steppedIndex = -1 ;

    float t = 0;
    VoxelCoord p_voxel_actual = point_to_voxel_center(xyz_w(0), xyz_w(1), xyz_w(2));
    //std::cout<<"voxelSize is "<<voxelSize_<<"Starting position "<<p.transpose()<<", ending position "<<p_max.transpose()<<", actual depth "<<xyz_w.transpose()
    //         <<", actual voxel coord is "<<p_voxel_actual.xc<<", "<<p_voxel_actual.yc<<", "<<p_voxel_actual.zc<<"\n";
    while (t < maxDist) {
      //Eigen::Vector3f curr_p = p + dir * t;
      //p_voxel_actual = point_to_voxel_center(curr_p(0), curr_p(1), curr_p(2));
      /// std::cout<<"Current Point "<<curr_p.transpose()<< ", query voxel "<<ix<<", "<<iy<<", "<<iz<<", t="<<t<<", raw position is "<<xyz_w.transpose()
      //         <<", actual voxel coord is "<<p_voxel_actual.xc<<", "<<p_voxel_actual.yc<<", "<<p_voxel_actual.zc
      //         <<std::endl;              
      const Voxel<ActivePoint> * nextVoxel = query_point( ix, iy, iz);
      
      if (nextVoxel) {
        return nextVoxel;
      }
      if (txMax < tyMax) {
        if (txMax < tzMax) {
          ix += stepx;
          t = txMax;
          txMax += txDelta;
          // tyMax += txDelta * dir(1);
          //tzMax += txDelta * dir(2);
          //std::cout<<"Update tx, tx="<<txMax<<", ty="<<tyMax<<", tzMax="<<tzMax<<std::endl;
          //steppedIndex = 0;
        } else {
          iz += stepz;
          t = tzMax;
          tzMax += tzDelta;
          //std::cout<<"Update tz, tx="<<txMax<<", ty="<<tyMax<<", tzMax="<<tzMax<<std::endl;
          //steppedIndex = 2;
        }
      } else {
        if (tyMax < tzMax) {
          iy += stepy;
          t = tyMax;
          tyMax += tyDelta;
          //std::cout<<"Update ty, tx="<<txMax<<", ty="<<tyMax<<", tzMax="<<tzMax<<std::endl;          
          //steppedIndex = 1;
        } else {
          iz += stepz;
          t = tzMax;
          tzMax += tzDelta;
          // std::cout<<"Update tz, tx="<<txMax<<", ty="<<tyMax<<", tzMax="<<tzMax<<std::endl;          
          //steppedIndex = 2;
        }
      }


    }

    return nullptr;

    // raycast algo paper
    // http://www.cse.chalmers.se/edu/year/2010/course/TDA361/grid.pdf    
    // consider raycast vector to be parametrized by t
    //   vec = [px,py,pz] + t * [dx,dy,dz]

    
  }

  

  template <>
  void VoxelMap<ActivePoint>::query_point_raycasting(const ActivePoint * pt,
                                                     std::vector<const Voxel<ActivePoint>*> & occupied_voxels_along_ray,
                                                     float minDist, float maxDist) const {

    occupied_voxels_along_ray.clear();
    
    Eigen::Matrix4f Tcw = pt->reference()->camToWorld().matrix(); //camToWorld
    Eigen::Vector3f xyz_cam = pt->xyz();// = //Tcw.block<3,3>(0,0) * pt->xyz() + Tcw.block<3,1>(0,3);
    Eigen::Vector3f xyz_w = Tcw.block<3,3>(0,0) * pt->xyz() + Tcw.block<3,1>(0,3);
    Eigen::Vector3f p_c_normalized = xyz_cam * pt->iDepth();
    Eigen::Vector3f p_cam = p_c_normalized * minDist; // pt in cam frame normalized
    Eigen::Vector3f p_cam_max = p_c_normalized * maxDist;
    Eigen::Vector4f p_cam_4;
    p_cam_4 << p_cam(0), p_cam(1), p_cam(2), 1.0;
    Eigen::Vector3f p = (Tcw * p_cam_4).head<3>();
    Eigen::Vector3f p_max = Tcw.block<3,3>(0,0) * p_cam_max + Tcw.block<3,1>(0,3);

    
    Eigen::Vector3f dir = (p_max - p).normalized();  //(Tcw.block<3,3>(0,0) * (p_max - p_cam)).normalized(); //''Eigen::Map<Eigen::Vector3f>(dirArr);
    
    //int ix = std::floor(p(0));// | 0
    //int iy = std::floor(p(1));// | 0
    //int iz = std::floor(p(2));// | 0
    VoxelCoord p_voxel = point_to_voxel_center(p(0), p(1), p(2));
    float ix = p_voxel.xc;
    float iy = p_voxel.yc;
    float iz = p_voxel.zc;
      
 
    float stepx = ((dir(0) > 0) ? voxelSize_ : -voxelSize_);
    float stepy = ((dir(1) > 0) ? voxelSize_ : -voxelSize_);
    float stepz = ((dir(2) > 0) ? voxelSize_ : -voxelSize_);
    
    float txDelta = std::abs(voxelSize_ / dir(0));
    float tyDelta = std::abs(voxelSize_ / dir(1));
    float tzDelta = std::abs(voxelSize_ / dir(2));
    
    float xdist = (stepx > 0) ? (ix + voxelSize_ /2 - p(0)) : ( ix - voxelSize_/2 - p(0) );
    float ydist = (stepy > 0) ? (iy + voxelSize_ /2 - p(1)) : ( iy - voxelSize_/2 - p(1));
    float zdist = (stepz > 0) ? (iz + voxelSize_ /2 - p(2)) : ( iz - voxelSize_/2 - p(2));    

    //float txMax = (txDelta < std::numeric_limits<float>::max()) ? txDelta * xdist : std::numeric_limits<float>::max();
    //float tyMax = (tyDelta < std::numeric_limits<float>::max()) ? tyDelta * ydist : std::numeric_limits<float>::max();
    //float tzMax = (tzDelta <  std::numeric_limits<float>::max()) ? tzDelta * zdist :  std::numeric_limits<float>::max();
    float txMax = (txDelta < std::numeric_limits<float>::max()) ? xdist / dir(0) : std::numeric_limits<float>::max();
    float tyMax = (tyDelta < std::numeric_limits<float>::max()) ? ydist / dir(1) : std::numeric_limits<float>::max();
    float tzMax = (tzDelta <  std::numeric_limits<float>::max()) ? zdist / dir(2) :  std::numeric_limits<float>::max();
        
    int steppedIndex = -1 ;

    float t = 0;
    VoxelCoord p_voxel_actual = point_to_voxel_center(xyz_w(0), xyz_w(1), xyz_w(2));
    //std::cout<<"voxelSize is "<<voxelSize_<<"Starting position "<<p.transpose()<<", ending position "<<p_max.transpose()<<", actual depth "<<xyz_w.transpose()
    //         <<", actual voxel coord is "<<p_voxel_actual.xc<<", "<<p_voxel_actual.yc<<", "<<p_voxel_actual.zc<<"\n";
    while (t < maxDist) {
      //Eigen::Vector3f curr_p = p + dir * t;
      //p_voxel_actual = point_to_voxel_center(curr_p(0), curr_p(1), curr_p(2));
      /// std::cout<<"Current Point "<<curr_p.transpose()<< ", query voxel "<<ix<<", "<<iy<<", "<<iz<<", t="<<t<<", raw position is "<<xyz_w.transpose()
      //         <<", actual voxel coord is "<<p_voxel_actual.xc<<", "<<p_voxel_actual.yc<<", "<<p_voxel_actual.zc
      //         <<std::endl;              
      const Voxel<ActivePoint> * nextVoxel = query_point( ix, iy, iz);
      
      if (nextVoxel) {
        //return nextVoxel;
        occupied_voxels_along_ray.push_back(nextVoxel);
      } 
      if (txMax < tyMax) {
        if (txMax < tzMax) {
          ix += stepx;
          t = txMax;
          txMax += txDelta;
          // tyMax += txDelta * dir(1);
          //tzMax += txDelta * dir(2);
          //std::cout<<"Update tx, tx="<<txMax<<", ty="<<tyMax<<", tzMax="<<tzMax<<std::endl;
          //steppedIndex = 0;
        } else {
          iz += stepz;
          t = tzMax;
          tzMax += tzDelta;
          //std::cout<<"Update tz, tx="<<txMax<<", ty="<<tyMax<<", tzMax="<<tzMax<<std::endl;
          //steppedIndex = 2;
        }
      } else {
        if (tyMax < tzMax) {
          iy += stepy;
          t = tyMax;
          tyMax += tyDelta;
          //std::cout<<"Update ty, tx="<<txMax<<", ty="<<tyMax<<", tzMax="<<tzMax<<std::endl;          
          //steppedIndex = 1;
        } else {
          iz += stepz;
          t = tzMax;
          tzMax += tzDelta;
          // std::cout<<"Update tz, tx="<<txMax<<", ty="<<tyMax<<", tzMax="<<tzMax<<std::endl;          
          //steppedIndex = 2;
        }
      }


    }

    return;

    // raycast algo paper
    // http://www.cse.chalmers.se/edu/year/2010/course/TDA361/grid.pdf    
    // consider raycast vector to be parametrized by t
    //   vec = [px,py,pz] + t * [dx,dy,dz]

    
  }

  template <>
  bool VoxelMap<ActivePoint>::insert_point_raytracing_with_free_points(ActivePoint * pt) {

    float minDist = 0;
    float maxDist = 1.0/pt->iDepth();
    Eigen::Matrix4f Tcw = pt->reference()->camToWorld().matrix(); //camToWorld
    Eigen::Vector3f xyz_cam = pt->xyz();// = //Tcw.block<3,3>(0,0) * pt->xyz() + Tcw.block<3,1>(0,3);
    Eigen::Vector3f xyz_w = Tcw.block<3,3>(0,0) * pt->xyz() + Tcw.block<3,1>(0,3);
    Eigen::Vector3f p_c_normalized = xyz_cam * pt->iDepth();
    Eigen::Vector3f p_cam = p_c_normalized * minDist; // pt in cam frame normalized
    Eigen::Vector3f p_cam_max = p_c_normalized * maxDist;
    Eigen::Vector4f p_cam_4;
    p_cam_4 << p_cam(0), p_cam(1), p_cam(2), 1.0;
    Eigen::Vector3f p = (Tcw * p_cam_4).head<3>();
    Eigen::Vector3f p_max = Tcw.block<3,3>(0,0) * p_cam_max + Tcw.block<3,1>(0,3);

    
    Eigen::Vector3f dir = (p_max - p).normalized();  //(Tcw.block<3,3>(0,0) * (p_max - p_cam)).normalized(); //''Eigen::Map<Eigen::Vector3f>(dirArr);
    
    //int ix = std::floor(p(0));// | 0
    //int iy = std::floor(p(1));// | 0
    //int iz = std::floor(p(2));// | 0
    VoxelCoord p_voxel = point_to_voxel_center(p(0), p(1), p(2));
    float ix = p_voxel.xc;
    float iy = p_voxel.yc;
    float iz = p_voxel.zc;
      
 
    float stepx = ((dir(0) > 0) ? voxelSize_ : -voxelSize_);
    float stepy = ((dir(1) > 0) ? voxelSize_ : -voxelSize_);
    float stepz = ((dir(2) > 0) ? voxelSize_ : -voxelSize_);
    
    float txDelta = std::abs(voxelSize_ / dir(0));
    float tyDelta = std::abs(voxelSize_ / dir(1));
    float tzDelta = std::abs(voxelSize_ / dir(2));
    
    float xdist = (stepx > 0) ? (ix + voxelSize_ /2 - p(0)) : ( ix - voxelSize_/2 - p(0) );
    float ydist = (stepy > 0) ? (iy + voxelSize_ /2 - p(1)) : ( iy - voxelSize_/2 - p(1));
    float zdist = (stepz > 0) ? (iz + voxelSize_ /2 - p(2)) : ( iz - voxelSize_/2 - p(2));    

    //float txMax = (txDelta < std::numeric_limits<float>::max()) ? txDelta * xdist : std::numeric_limits<float>::max();
    //float tyMax = (tyDelta < std::numeric_limits<float>::max()) ? tyDelta * ydist : std::numeric_limits<float>::max();
    //float tzMax = (tzDelta <  std::numeric_limits<float>::max()) ? tzDelta * zdist :  std::numeric_limits<float>::max();
    float txMax = (txDelta < std::numeric_limits<float>::max()) ? xdist / dir(0) : std::numeric_limits<float>::max();
    float tyMax = (tyDelta < std::numeric_limits<float>::max()) ? ydist / dir(1) : std::numeric_limits<float>::max();
    float tzMax = (tzDelta <  std::numeric_limits<float>::max()) ? zdist / dir(2) :  std::numeric_limits<float>::max();
        
    int steppedIndex = -1 ;

    float t = 0;
    VoxelCoord p_voxel_actual = point_to_voxel_center(xyz_w(0), xyz_w(1), xyz_w(2));
    //std::cout<<"voxelSize is "<<voxelSize_<<"Starting position "<<p.transpose()<<", ending position "<<p_max.transpose()<<", actual depth "<<xyz_w.transpose()
    //         <<", actual voxel coord is "<<p_voxel_actual.xc<<", "<<p_voxel_actual.yc<<", "<<p_voxel_actual.zc<<"\n";
    while (t < maxDist) {
      //Eigen::Vector3f curr_p = p + dir * t;
      //p_voxel_actual = point_to_voxel_center(curr_p(0), curr_p(1), curr_p(2));
      /// std::cout<<"Current Point "<<curr_p.transpose()<< ", query voxel "<<ix<<", "<<iy<<", "<<iz<<", t="<<t<<", raw position is "<<xyz_w.transpose()
      //         <<", actual voxel coord is "<<p_voxel_actual.xc<<", "<<p_voxel_actual.yc<<", "<<p_voxel_actual.zc
      //         <<std::endl;              
      const Voxel<ActivePoint> * nextVoxel = query_point( ix, iy, iz);
      
      if (nextVoxel) {
        //return nextVoxel;
        break;
        //occupied_voxels_along_ray.push_back(nextVoxel);
      } 
      if (txMax < tyMax) {
        if (txMax < tzMax) {
          ix += stepx;
          t = txMax;
          txMax += txDelta;
          // tyMax += txDelta * dir(1);
          //tzMax += txDelta * dir(2);
          //std::cout<<"Update tx, tx="<<txMax<<", ty="<<tyMax<<", tzMax="<<tzMax<<std::endl;
          //steppedIndex = 0;
        } else {
          iz += stepz;
          t = tzMax;
          tzMax += tzDelta;
          //std::cout<<"Update tz, tx="<<txMax<<", ty="<<tyMax<<", tzMax="<<tzMax<<std::endl;
          //steppedIndex = 2;
        }
      } else {
        if (tyMax < tzMax) {
          iy += stepy;
          t = tyMax;
          tyMax += tyDelta;
          //std::cout<<"Update ty, tx="<<txMax<<", ty="<<tyMax<<", tzMax="<<tzMax<<std::endl;          
          //steppedIndex = 1;
        } else {
          iz += stepz;
          t = tzMax;
          tzMax += tzDelta;
          // std::cout<<"Update tz, tx="<<txMax<<", ty="<<tyMax<<", tzMax="<<tzMax<<std::endl;          
          //steppedIndex = 2;
        }
      }


    }
    
        // 1. find coresponding voxel coordinates
    VoxelCoord intCoord = point_to_voxel_center(pt);
    //std::cout<<"insert the point to "<<intCoord.xc<<", "<<intCoord.yc<<", "<<intCoord.zc<<"\n";    
    // 2. insert point to map
    if (vmap_.count(intCoord)) {
      // voxel already exists
      // std::cout << "Existing voxel" << std::endl;
      std::vector<ActivePoint*>& voxPts = vmap_[intCoord].voxPoints;
      // Check if the point already exists
      for (auto it = voxPts.begin(); it != voxPts.end(); it++) {
        // TODO: make sure this equal is correct
        if (*it == pt) 
          return false; 
      }
      // add only if point didn't exist
      vmap_[intCoord].voxPoints.push_back(pt);
    } else {
      // voxel didn't exist, create voxel and add the point
      vmap_[intCoord] = Voxel<ActivePoint>(intCoord.xc, intCoord.yc, intCoord.zc);
      vmap_[intCoord].voxPoints.push_back(pt);
    }
    // std::cout << "=============================================\n";
    return true;

  }  
  

  //updateCovis debug use
  template <typename PointType>
  void VoxelMap<PointType>::save_voxels_pcd(std::string filename) const {
      pcl::PointCloud<pcl::PointXYZ> pc;
      for (const auto& voxelPair : vmap_) {
          const VoxelCoord& vc = voxelPair.first;
          pcl::PointXYZ p;
          p.x = vc.xc;
          p.y = vc.yc;
          p.z = vc.zc;
          pc.push_back(p);
      }
      pcl::io::savePCDFile(filename, pc);
      std::cout << "Wrote voxel centers to " << filename << std::endl;
  }

  template <>
  void VoxelMap<ActivePoint>::save_points_pcd(std::string filename) const {
    pcl::PointCloud<pcl::PointXYZ> pc;
    for (const auto& voxelPair : vmap_) {
      for (ActivePoint* pt : voxelPair.second.voxPoints) {
        // get pt coord in world frame
        Eigen::Matrix4f Tcw = pt->reference()->camToWorld().matrix(); //camToWorld
        Eigen::Vector3f p_cam = pt->xyz(); // pt in cam frame
        Eigen::Vector4f p_cam_4;
        p_cam_4 << p_cam(0), p_cam(1), p_cam(2), 1.0;
        Eigen::Vector4f p_wld = Tcw * p_cam_4;
        //             // push to pcl cloud
        pcl::PointXYZ p;
        p.x = p_wld(0);
        p.y = p_wld(1);
        p.z = p_wld(2);
        pc.push_back(p);
      }
    }
    pcl::io::savePCDFile(filename, pc);
    std::cout << "Wrote voxel centers to " << filename << std::endl;
  }

  template class Voxel<ActivePoint>;
  template class VoxelMap<ActivePoint>;

  template class Voxel<SimplePoint>;
  template class VoxelMap<SimplePoint>;

  
}
