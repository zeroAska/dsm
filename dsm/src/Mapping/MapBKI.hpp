#pragma once
#include <vector>

#include "Mapping/Map.hpp"
#include "Mapping/bkioctomap.h"

namespace dsm {
  class MapBKI : public Map<std::vector, > {
    MapBKI() {
      
    }
    
    virtual void insert_pointcloud() {
      
    }

  private:
    semantic_bki::SemanticBKIOctomap map_;
  };
}
