#include <iostream>
#include <vector>

#include "agritech_manip/static_object_tracker.h"

void printObjects(const StaticObjectTracker::VectorObject& objects,
                  std::ostream& out);

int main(int argc, char** argv) {
    StaticObjectTracker tracker;
    StaticObjectTracker::VectorObject objectsAll;
    StaticObjectTracker::VectorObject objectsObserved;
    StaticObjectTracker::VectorObject objectsValid;
    StaticObjectTracker::Object otmp;

    std::cout << "Setting tracker" << std::endl;
    tracker.setAssociationDistance(0.6);
    tracker.setHitMissBounds(3, 2);

    // Object 0
    otmp.center(0) = 1.0;
    otmp.center(1) = -2.0;
    otmp.center(2) = 3.0;
    objectsAll.push_back(otmp);

    // Object 1
    otmp.center(0) = 3.0;
    otmp.center(1) = 2.0;
    otmp.center(2) = -1.0;
    objectsAll.push_back(otmp);

    // Object 2
    otmp.center(0) = -3.0;
    otmp.center(1) = 3.0;
    otmp.center(2) = 0.0;
    objectsAll.push_back(otmp);

    // Object 3
    otmp.center(0) = 0.0;
    otmp.center(1) = 4.0;
    otmp.center(2) = 4.0;
    objectsAll.push_back(otmp);

    for (int iter = 0; iter < 10; ++iter) {
        std::cout << "\n----\niteration " << iter << "\n";
        objectsObserved.clear();
        for (size_t i = 0; i < objectsAll.size(); ++i) {
            if (i == 0) {
                if (iter % 3 != 0)
                    objectsObserved.push_back(objectsAll[i]);
            } else if (i == 2) {
                if (2 <= iter && iter <= 6)
                    objectsObserved.push_back(objectsAll[i]);
            } else {
                objectsObserved.push_back(objectsAll[i]);
            }
            if (!objectsObserved.empty()) {
                objectsObserved.back().center(0) +=
                    ((iter + i) % 2 ? 0.01 : -0.01) * (iter % 4 + 1);
                objectsObserved.back().center(1) +=
                    (iter % 2 ? 0.01 : -0.01) * ((iter + i) % 3 + 1);
                objectsObserved.back().center(2) +=
                    (iter % 2 ? 0.02 : -0.02) * ((iter + i) % 5 + 1);
            }
        }
        std::cout << "observed objects:\n";
        printObjects(objectsObserved, std::cout);

        tracker.update(objectsObserved);
        std::cout << "all tracked objects:\n";
        printObjects(tracker.getObjects(), std::cout);
        std::cout << "valid tracked objects:\n";
        tracker.getValidObjects(objectsValid);
        printObjects(objectsValid, std::cout);
    }

    return 0;
}

void printObjects(const StaticObjectTracker::VectorObject& objects,
                  std::ostream& out) {
    for (auto& o : objects) {
        out << "  [" << o.center(0) << "," << o.center(1) << "," << o.center(2)
            << "] hitNum " << o.hitNum << " missNum " << o.missNum << "\n";
    }
}