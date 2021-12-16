#include "DataStructures/VoxelMap.h"

namespace dsm {

    VoxelMap::VoxelMap(float voxelSize) : voxelSize_(voxelSize) {
        std::cout << "Assigned voxelSize_" << voxelSize << std::endl;
    }

    bool VoxelMap::insert_point(Point pt) {
        // std::cout << "integer point coord: " << pt.x << ", " << pt.y << ", " << pt.z << std:: endl;
        // 1. convert to integer coordinates
        VoxelCoord intCoord = point_to_voxel_center(pt);
        // std::cout << "=============================================\n";
        // std::cout << "integer point coord: " << intCoord.xc << ", " << intCoord.yc << ", " << intCoord.zc << std:: endl;
        // Debug: print its hash key
        // std::cout << "Hashed: " << std::hash<VoxelCoord>{}(intCoord) << std::endl;
        // 2. insert point to map
        if (vmap_.count(intCoord)) {
            // voxel already exists
            // std::cout << "Existing voxel" << std::endl;
            std::vector<Point>& voxPts = vmap_[intCoord].voxPoints;
            // Check if the point already exists
            for (auto it = voxPts.begin(); it != voxPts.end(); it++) {
                if (*it == pt)
                    return false;
            }
            // add only if point didn't exist
            vmap_[intCoord].voxPoints.push_back(pt);
        } else {
            // voxel didn't exist, create voxel and add the point
            vmap_[intCoord] = Voxel(intCoord.xc, intCoord.yc, intCoord.zc);
            vmap_[intCoord].voxPoints.push_back(pt);
        }
        // std::cout << "=============================================\n";
        return true;
    }

    bool VoxelMap::delete_point(Point pt) {
        // 1. convert to integer coord to look up its voxel
        VoxelCoord intCoord = point_to_voxel_center(pt);
        if (!vmap_.count(intCoord))
            return false;
        // 2. remove this point from the voxel
        std::vector<Point>& curVoxPts = vmap_[intCoord].voxPoints;
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

    VoxelMap::~VoxelMap() {
        std::cout<<"Voxel map destructed\n";
    }

    bool VoxelMap::query_point(Point pt, Voxel*& contained_voxel) {
        // 1. convert to integer coord to look up its voxel
        VoxelCoord intCoord = point_to_voxel_center(pt);
        if (!vmap_.count(intCoord))
            return false;
        // std::cout << vmap_[intCoord].xc << ", " << vmap_[intCoord].yc << ", " << vmap_[intCoord].zc << std::endl;
        contained_voxel = &vmap_[intCoord];
        return true;
    }

    std::vector<int> VoxelMap::voxel_seen_frames(VoxelCoord q_vox) {
        std::unordered_set<int> resSet;
        const std::vector<Point>& vxPts = vmap_[q_vox].voxPoints;
        for (const Point& p : vxPts) {
            resSet.insert(p.frameID);
        }
        return std::vector<int>(resSet.begin(), resSet.end());
    }

    size_t VoxelMap::size() {
        return vmap_.size();
    }

    VoxelCoord VoxelMap::point_to_voxel_center(Point pt) {
        std::vector<float> res(3);
        std::vector<float> orig = {pt.x, pt.y, pt.z};
        for (int i = 0; i < 3; i++) {
            // find remainder
            float rem = fmod(orig[i], voxelSize_);
            int addOne = 0;
            if (rem >= 0.0)
                addOne = rem > (voxelSize_ / 2.0f);
            else 
                addOne = -(rem < (voxelSize_ / 2.0f));
            res[i] = (int(orig[i] / voxelSize_) + addOne) * voxelSize_;
        }
        VoxelCoord resCoord{res[0], res[1], res[2]};
        return resCoord;
    }
}