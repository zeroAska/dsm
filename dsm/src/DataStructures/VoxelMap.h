#pragma once

#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <functional>
#include <ostream>
#include <vector>
#include <math.h>

#include <Eigen/Core>

#include "FullSystem/DSMLib.h"
#include "ActivePoint.h"

#include "sophus/se3.hpp"

namespace dsm
{
	class Frame;
	class ActivePoint;
    struct SimplePoint {
        float x;
        float y;
        float z;
        int pixelIdx;

        SimplePoint(float xIn, float yIn, float zIn) : SimplePoint(xIn, yIn, zIn, -1)  {}

        SimplePoint(float xIn, float yIn, float zIn, int pixelIdxIn) : x(xIn), y(yIn), z(zIn), pixelIdx(pixelIdxIn) {}

        int32_t currentID() const {
            //dummy function
            std::cerr << "SimplePoint::currentID not implemented\n";
            return -1;
        }

        bool operator==(const SimplePoint& other) const {
            return (x == other.x && y == other.y && z == other.z && pixelIdx == other.pixelIdx);
        }

        friend std::ostream& operator<<(std::ostream& os, const SimplePoint& pt) {
            os << "x: " << pt.x << ", y: " << pt.y << ", z: " << pt.z;
            return os;
        }
    };

    // used as hash key in voxelMap
    struct VoxelCoord {
        float xc;
        float yc;
        float zc;

        bool operator==(const VoxelCoord& other) const {
          return (std::lrint(100 * xc) == std::lrint(other.xc * 100)
                  && std::lrint(100 * yc) == std::lrint( other.yc * 100)
                  &&  std::lrint(100 * zc) == std::lrint(100*other.zc));
        }

        friend std::ostream& operator<<(std::ostream& os, const VoxelCoord& vc) {
            os << "x: " << vc.xc << ", y: " << vc.yc << ", z: " << vc.zc;
            return os;
        }
    };
}

template<>
struct std::hash<dsm::VoxelCoord> {
    std::size_t operator()(dsm::VoxelCoord const& vc) const {
        int p1 = 204803, p2 = 618637, p3 = 779189;
        return (static_cast<int>(std::lrint(vc.xc * 100 * p1))
                ^ static_cast<int>(std::lrint(vc.yc * 100 * p2))
                ^ static_cast<int>(std::lrint(vc.zc * 100 * p3)));
    }
};

namespace dsm {

    template <typename PointType>
    class Voxel {
    public:
        Voxel() {}

        Voxel(float x, float y, float z) : xc(x), yc(y), zc(z) {}
      Voxel(float x, float y, float z,
            int occupancy) : xc(x), yc(y), zc(z), occupancy(occupancy) {}

      void update_occupancy(int val) { occupancy+= val; }

      Eigen::Vector3f centroid() const {
        Eigen::Vector3f v;
        v << xc, yc, zc;
        return v;
      }
      
    public:
        // voxel center coordinates
        float xc;
        float yc;
        float zc;
        // points in the voxel
        std::vector<PointType*> voxPoints;

      int occupancy;

      const static int occupied_limit = 90;
      const static int free_limit = 30;
      const static int occupancy_step = 10;
    };

    template <typename PointType>
    class DSM_EXPORTS_DLL VoxelMap {
    public:
        VoxelMap(float voxelSize);
        ~VoxelMap(); 

        /**
         * @brief insert a 3D point into the voxel map
         * @param pt: a 3D point
         * @return true if insertion is successful; False if point already exists
         */
        bool insert_point(PointType* pt);
        bool insert_point_raytracing_with_free_points(PointType * pt);

        /**
         * @brief remove a 3D point from the voxel map
         * @param pt: a 3D point
         * @return true if deletion is successful; False if point doesn't exist in map
         */
        bool delete_point(PointType* pt);

        /**
         * @brief After BA, an ActivePoint's ref frame changes pose, need to provide its containing voxel for removal
         * @param pt: a 3D point
         * @param voxel: the voxel that stores the given pt
         * @return true if deletion is successful; False if point doesn't exist in map
         */
        bool delete_point_BA(PointType* pt, const Voxel<PointType>* voxel);

        /**
         * @brief query a 3D point to find the voxel containing it in the voxel map
         * @param pt: a 3D point
         * @return nullptr if no voxel exists at the given pt, or the voxel
         */
        const Voxel<PointType>* query_point(const PointType* pt) const;
        const Voxel<PointType>* query_point(float globalX, float globalY, float globalZ) const;
        void query_point_raycasting(const Eigen::Vector3f & p_local,
                                    const Sophus::SE3f & camToWorld,
                                    std::vector<const Voxel<PointType>*> & occupied_voxels_along_ray,                                    

                                    float minDist=0.5, float maxDist=55.0) const;

        const Voxel<PointType>* query_point_raycasting(const ActivePoint * pt, float minDist=0.1, float maxDist=55);
        
        
        template<typename MapT, typename QueryFunctor, typename ResultT>
        void query_point_raycasting(const Eigen::Vector3f & pt_local,
                                    const Sophus::SE3f & Tcw_sophus,
                                    float minDist, float maxDist,
                                    const MapT & map,
                                    std::vector<ResultT> & results,
                                    QueryFunctor query_function
                                    ) const ;
        
        
        //void query_point_raycasting(const PointType * pt,
        //                            std::vector<const Voxel<PointType>*> & occupied_voxels_along_ray,
        //                            float minDist=0.1, float maxDist=55.0) const;
        

        /**
         * @brief obtain the frameIds that have seen this voxel
         * @param pt: a 3D point
         * @return a set of frameIds, empty if point isn't inside a voxel
         */
        std::unordered_set<int> voxel_seen_frames(PointType* pt) const;

        /**
         * @brief returns the number of voxels
         * @return number of voxels in the voxelmap
         */
        size_t size();

        /**
         * @brief takes one point from every exisitng voxel
         * @return a vector of points
         */
        const std::vector<PointType*> sample_points() const;


    // updateCovis Debug use
    // public:
         void save_voxels_pcd(std::string filename) const;

         void save_points_pcd(std::string filename) const;

    private:
        /**
         * @brief finds the voxel center coordinate that contains a given point
         * @param pt: a 3D point
         * @return the coordinate of the voxel center
         */
        VoxelCoord point_to_voxel_center(const PointType* pt) const;
        VoxelCoord point_to_voxel_center(float globalX, float globalY, float globalZ) const;

    private:
        float voxelSize_ = 0.1f;                               // Edge length of a single voxel cubic
        std::unordered_map<VoxelCoord, Voxel<PointType>> vmap_;    // Hash map that stores the full voxel map

    };

  /*
    template <>
    VoxelCoord VoxelMap<ActivePoint>::point_to_voxel_center(const ActivePoint* pt) const;

    template <>
    const Voxel<ActivePoint>* VoxelMap<ActivePoint>::query_point_raycasting(const ActivePoint * pt, float minDist, float maxDist);

    template <>
    void VoxelMap<ActivePoint>::save_points_pcd(std::string filename) const;

    template <>
    VoxelCoord VoxelMap<SimplePoint>::point_to_voxel_center(const SimplePoint* pt) const;
  */

}
