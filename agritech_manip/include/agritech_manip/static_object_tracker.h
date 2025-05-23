#ifndef STATIC_OBJECT_TRACKER_H_
#define STATIC_OBJECT_TRACKER_H_
  
#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
#include <vector>

class StaticObjectTracker {
   public:
    struct Object {
        // double x;
        // double y;
        // double z;
        Eigen::Vector3d center;
        int hitNum;
        int missNum;

        // Object() : x(0.0), y(0.0), z(0.0), hitNum(0), missNum(0) {}
        Object() : center(Eigen::Vector3d::Zero()), hitNum(0), missNum(0) {}
    };
    using VectorObject = typename std::vector<Object>;

    /**
     * @brief Default constructor.
     */
    StaticObjectTracker();

    /**
     * @brief Destructor.
     */
    ~StaticObjectTracker();

    /**
     * @brief Sets the hysteresis parameters required to add or remove a valid
     * object.
     * @param hitNumMin minimum number of observation to classify an object as
     * valid
     * @param missNumMax maximum number of consecutive miss to classify an
     * object as failed.
     */
    void setHitMissBounds(int hitNumMin, int missNumMax);

    /**
     * @brief Set the maximum distance for association.
     */
    void setAssociationDistance(double associationDist);

    /**
     * @brief Updates the internal list of objects using last observation.
     */
    void update(const std::vector<Object>& observed);

    /**
     * @brief Returns a const reference to the internal list of objects.
     */
    const VectorObject& getObjects();

    /**
     * @brief Returns a const reference to the internal list of objects.
     */
    void getValidObjects(VectorObject& valids);

    /**
     * @brief Resets the counters hitNum and missNum of each object.
     *
     */
    void resetCounters();

    /**
     * @brief Associates observed objects to the internally tracked ones.
     * This method is used internally.
     *
     * @param observed
     * @param associationIdx
     */
    void associateNN(const VectorObject& observed,
                     std::vector<int>& associationIdx);

   private:
    VectorObject objects_;
    int hitNumMin_;
    int missNumMax_;
    double associationDist_;
};

#endif