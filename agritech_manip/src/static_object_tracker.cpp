#include "agritech_manip/static_object_tracker.h"
#include <cmath>

StaticObjectTracker::StaticObjectTracker()
    : objects_(), hitNumMin_(1), missNumMax_(1) {}

StaticObjectTracker::~StaticObjectTracker() {}

void StaticObjectTracker::setHitMissBounds(int hitNumMin, int missNumMax)
{
    hitNumMin_ = hitNumMin;
    missNumMax_ = missNumMax;
}

void StaticObjectTracker::setAssociationDistance(double associationDist)
{
    associationDist_ = associationDist;
}

void StaticObjectTracker::update(const std::vector<Object> &observed)
{
    std::vector<int> associationIdx;
    std::vector<bool> objectObserved(objects_.size(), false);
    int objectIdx;

    // Associates each observed object to each of the tracked object.
    // TODO: implements a better association policy.
    associateNN(observed, associationIdx);

    for (size_t i = 0; i < associationIdx.size(); ++i)
    {
        // Increments the stats for each associated object or adds the new
        // object if it is unassociated
        objectIdx = associationIdx[i];
        if (0 <= objectIdx && objectIdx < (int)objects_.size())
        {
            objectObserved[objectIdx] = true;
            // if (objects_[objectIdx].hitNum <= hitNumMin_)
            objects_[objectIdx].center = (objects_[objectIdx].center * objects_[objectIdx].hitNum + observed[i].center) / (objects_[objectIdx].hitNum+1);
            objects_[objectIdx].hitNum++;
            objects_[objectIdx].missNum = 0;
        }
        else
        {
            objects_.push_back(observed[i]);
            objects_.back().hitNum = 1;
            objects_.back().missNum = 0;
        }
    }

    // Updates the stats of unobserved objects and removes those with larg miss
    // counter
    for (size_t i = 0; i < objectObserved.size(); ++i)
    {
        if (!objectObserved[i])
        {
            objects_[i].missNum++;
        }
    }
    objects_.erase(std::remove_if(objects_.begin(), objects_.end(),
                                  [&](const Object &o) -> bool
                                  {
                                      return (o.missNum >= missNumMax_);
                                  }),
                   objects_.end());
}

const StaticObjectTracker::VectorObject &StaticObjectTracker::getObjects()
{
    return objects_;
}

void StaticObjectTracker::getValidObjects(VectorObject &valids)
{
    valids.clear();
    for (auto &object : objects_)
    {
        if (object.hitNum >= hitNumMin_)
        {
            valids.push_back(object);
        }
    }
}

void StaticObjectTracker::resetCounters()
{
    for (auto &object : objects_)
    {
        object.hitNum = 0;
        object.missNum = 0;
    }
}

void StaticObjectTracker::associateNN(const VectorObject &observed,
                                      std::vector<int> &associationIdx)
{
    double dist, distMin; // dx, dy, dz;
    int jmin;
    associationIdx.resize(observed.size());
    for (size_t i = 0; i < observed.size(); ++i)
    {
        jmin = -1;
        distMin = 1e+9;
        for (size_t j = 0; j < objects_.size(); ++j)
        {
            // dx = objects_[j].x - observed[i].x;
            // dy = objects_[j].y - observed[i].y;
            // dz = objects_[j].z - observed[i].z;
            // dist = sqrt(dx * dx + dy * dy + dz * dz);
            dist = (objects_[j].center - observed[i].center).norm();
            if (jmin < 0 || dist < distMin)
            {
                jmin = j;
                distMin = dist;
            }
        }
        if (distMin < associationDist_)
        {
            associationIdx[i] = jmin;
        }
        else
        {
            associationIdx[i] = -1;
        }
    }
}

// void associateBB(const std::vector<Object>& observed,
//                  std::vector<int>& associationIdx) {}

// void associateBBImpl(const std::vector<Object>& observed,
//                      std::vector<int>& assocZX,
//                      std::vector<int>& assocXZ,
//                      size_t startIdx) {
//     for ()
// }