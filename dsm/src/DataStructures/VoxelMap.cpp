#include "VoxelMap.h"
#include "Frame.h"

namespace dsm {

    VoxelMap::VoxelMap(float voxelSize) : voxelSize_(voxelSize) {
        // std::cout << "Assigned voxelSize_" << voxelSize << std::endl;
    }

    bool VoxelMap::insert_point(ActivePoint* pt) {
        // 1. find coresponding voxel coordinates
        VoxelCoord intCoord = point_to_voxel_center(pt);
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
            vmap_[intCoord] = Voxel(intCoord.xc, intCoord.yc, intCoord.zc);
            vmap_[intCoord].voxPoints.push_back(pt);
        }
        // std::cout << "=============================================\n";
        return true;
    }

    bool VoxelMap::delete_point(ActivePoint* pt) {
        // 1. convert to integer coord to look up its voxel
        VoxelCoord intCoord = point_to_voxel_center(pt);
        if (!vmap_.count(intCoord))
            return false;
        // 2. remove this point from the voxel
        std::vector<ActivePoint*>& curVoxPts = vmap_[intCoord].voxPoints;
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

    const Voxel* VoxelMap::query_point(ActivePoint* pt) const {
        // 1. convert to integer coord to look up its voxel
        VoxelCoord intCoord = point_to_voxel_center(pt);
        if (!vmap_.count(intCoord))
            return nullptr;
        // std::cout << vmap_[intCoord].xc << ", " << vmap_[intCoord].yc << ", " << vmap_[intCoord].zc << std::endl;
        return &vmap_.at(intCoord);
    }

    std::unordered_set<int> VoxelMap::voxel_seen_frames(ActivePoint* pt) const {
        std::unordered_set<int> resSet;
        const Voxel* curVoxel = query_point(pt);
        if (curVoxel == nullptr)
            return resSet;
        for (const ActivePoint* p : curVoxel->voxPoints) {
            resSet.insert(p->currentID());
        }
        return resSet;
    }

    size_t VoxelMap::size() {
        return vmap_.size();
    }

    VoxelCoord VoxelMap::point_to_voxel_center(ActivePoint* pt) const {
        // 1. get pt coord in world frame
        Eigen::Matrix4f Tcw = pt->reference()->camToWorld().matrix(); //camToWorld
        Eigen::Vector3f p_cam = pt->xyz(); // pt in cam frame
        Eigen::Vector4f p_cam_4;
        p_cam_4 << p_cam(0), p_cam(1), p_cam(2), 1.0;
        Eigen::Vector4f p_wld = Tcw * p_cam_4;
        // 2. find its corresponding voxel
        std::vector<float> res(3);
        std::vector<float> orig = {p_wld(0), p_wld(1), p_wld(2)};
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