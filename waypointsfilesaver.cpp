#include "waypointsfilesaver.hpp"
#include <sstream>
#include <ctime>
#include <iomanip>
#include "pathexecutor.hpp" // for Waypoint definition
#include <iostream>
WaypointsFileSaver::WaypointsFileSaver()
{


    //  openFile();

}




WaypointsFileSaver WaypointsFileSaver::waypointsFileSaver;

void WaypointsFileSaver::writeString( std::stringstream & entry)
{
    myfile << entry.rdbuf()<<std::endl;
}



void WaypointsFileSaver::openFile()
{

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream ss;
    ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S.vsmlog");
   std::string  filename = ss.str();
    myfile= std::ofstream(filename);
    //myfile = std::fstream(filename, std::ios::out);// | std::ios::binary);
    this->fileName =filename;

}

bool WaypointsFileSaver::readStoredPoints(std::vector<Waypoint> *wpts)
{
fileName = "waypoints.txt";
    // try to open existing waypoint file
    std::ifstream istrm(fileName, std::ios::binary);
    if (!istrm.is_open()) {
        std::cout << "failed to open " << fileName << '\n';
        return false;
    }
    hasStoredWaypoints = true;
    // read points
    std::string str;
    int counter=0;
    while (std::getline(istrm, str))
    {

        std::istringstream iss(str);
        Position2DGPS p;
        Waypoint wp;

        if ((iss >> p.lon>>p.lat>>p.yaw>>wp.dwellTimeSec)) {
            wp.trajectory.push_back(Position2D(p.lon, p.lat,p.yaw) );
            wpts->push_back(wp);
        }else{// cant parse file
            std::cout<<"cant parse file, check if format is four doubles in a row seperated with space"<<std::endl;
            return false;
        }
        counter++;
    }
    std::cout<<"Parsed "<<counter<<" lines"<<std::endl;

}

bool WaypointsFileSaver::savePoints(std::vector<Waypoint> wpts)
{
    std::ofstream of(fileName);
    if (!of.is_open()) {
        std::cout << "failed to open " << fileName << "for writing"<<std::endl;
        return false;
    }
    std::stringstream ss;

    for (int i = 0; i < wpts.size(); ++i) {
        int last = wpts.at(i).trajectory.size()-1;
        ss<<wpts.at(i).trajectory.at(last).x<<" "<<wpts.at(i).trajectory.at(last).y<<" "<<wpts.at(i).trajectory.at(last).yaw<<" "<<wpts.at(i).dwellTimeSec<<std::endl;
    }
    of<<ss.rdbuf();
    of.close();
    return true;
}

bool WaypointsFileSaver::saveAddedPoints()
{
    savePoints(waypointsToSave);
}

bool WaypointsFileSaver::closeFile()
{
    getCurrentFileSize(); //writes file size in fileSize field for possible later access
    myfile.close();
    return (myfile.fail());

}

