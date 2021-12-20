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


namespace dsm
{
	class Frame;
	class ActivePoint;
    // struct Point {
    //     float x;
    //     float y;
    //     float z;
    //     int frameID;

    //     Point(float xIn, float yIn, float zIn) : Point(xIn, yIn, zIn, -1)  {}

    //     Point(float xIn, float yIn, float zIn, int frameIDIn) : x(xIn), y(yIn), z(zIn), frameID(frameIDIn) {}

    //     bool operator==(const Point& other) const {
    //         return (x == other.x && y == other.y && z == other.z && frameID == other.frameID);
    //     }

    //     friend std::ostream& operator<<(std::ostream& os, const Point& pt) {
    //         os << "x: " << pt.x << ", y: " << pt.y << ", z: " << pt.z;
    //         return os;
    //     }
    // };

    // used as hash key in voxelMap
    struct VoxelCoord {
        float xc;
        float yc;
        float zc;

        bool operator==(const VoxelCoord& other) const {
            return (xc == other.xc && yc == other.yc && zc == other.zc);
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
        return (static_cast<int>(vc.xc * 100 * p1) ^ static_cast<int>(vc.yc * 100 * p2) ^ static_cast<int>(vc.zc * 100 * p3));
    }
};

namespace dsm {

    class Voxel {
    public:
        Voxel() {}

        Voxel(float x, float y, float z) : xc(x), yc(y), zc(z) {}

    public:
        // voxel center coordinates
        float xc;
        float yc;
        float zc;
        // points in the voxel
        std::vector<ActivePoint*> voxPoints;
    };

    class DSM_EXPORTS_DLL VoxelMap {
    public:
        VoxelMap(float voxelSize);
        ~VoxelMap(); 

        /**
         * @brief insert a 3D point into the voxel map
         * @param pt: a 3D point
         * @return true if insertion is successful; False if point already exists
         */
        bool insert_point(ActivePoint* pt);


        /**
         * @brief remove a 3D point from the voxel map
         * @param pt: a 3D point
         * @return true if deletion is successful; False if point doesn't exist in map
         */
        bool delete_point(ActivePoint* pt);

        /**
         * @brief query a 3D point to find the voxel containing it in the voxel map
         * @param pt: a 3D point
         * @param contained_voxel: the query result
         * @return true if query finds a valid voxel, false if such voxel doesn't exist in map
         */
        bool query_point(ActivePoint* pt, Voxel& contained_voxel);

        /**
         * @brief obtain the frameIds that have seen this voxel
         * @param q_vox: queried voxel
         * @return a vector of frameIds
         */
        std::vector<int> voxel_seen_frames(VoxelCoord q_vox);

        /**
         * @brief returns the number of voxels
         * @return number of voxels in the voxelmap
         */
        size_t size();

    private:
        /**
         * @brief finds the voxel center coordinate that contains a given point
         * @param pt: a 3D point
         * @return the coordinate of the voxel center
         */
        VoxelCoord point_to_voxel_center(ActivePoint* pt);

    private:
        float voxelSize_ = 0.05f;                               // Edge length of a single voxel cubic
        std::unordered_map<VoxelCoord, Voxel> vmap_;    // Hash map that stores the full voxel map

    };


}