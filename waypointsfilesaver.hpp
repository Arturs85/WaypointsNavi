#ifndef WAYPOINTSFILESAVER_HPP
#define WAYPOINTSFILESAVER_HPP

#include <vector>
#include <fstream>
#include <string.h>
#include "pathexecutor.hpp" // for Waypoint definition



class WaypointsFileSaver
{


public:
    WaypointsFileSaver();

static WaypointsFileSaver waypointsFileSaver;
   std::ofstream myfile;
    size_t fileSize;
    std::string fileName = "waypoints.txt";
bool hasStoredWaypoints = false;
std::vector<Waypoint> waypointsToSave;

    void openFile();
   bool readStoredPoints(std::vector<Waypoint> *wpts);
   bool savePoints(std::vector<Waypoint> wpts);
bool saveAddedPoints();
    inline void writeArrayToFile(char *array, int size)
    {
        myfile.write(array, size);

    }

    inline size_t getCurrentFileSize()
    {
        if(myfile.is_open())
            fileSize = myfile.tellp();

        return fileSize;
    }

    bool closeFile();

    void writeString(std::stringstream &entry);
};

#endif // WAYPOINTSFILESAVER_HPP

