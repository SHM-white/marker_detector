//
// Created by robin on 23-3-8.
//

#ifndef SRC_HISTORY_MANAGER_H
#define SRC_HISTORY_MANAGER_H

#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <fstream>
#include <numeric>

class Marker;

class history_manager {
public:
    const int maxLife = 10, thresh = 10;
    int roi_x, roi_y;
    std::vector<Marker> markers;

    void add_history(std::vector<Marker>& new_markers);
};


#endif //SRC_HISTORY_MANAGER_H
