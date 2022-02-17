#include "waypointsfilesaver.hpp"
#include <sstream>
#include <ctime>
#include <iomanip>
#include "pathexecutor.hpp" // for Waypoint definition
#include <iostream>
#include <dirent.h>
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

bool WaypointsFileSaver::readStoredPoints(std::vector<Waypoint> *wpts,std::string fileName)
{
   // fileName = "waypoints.txt";
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

        if ((iss >> p.lon>>p.lat>>p.yaw>>wp.dwellTimeSec>>wp.orientToYaw>>wp.triggerOutputPin)) {
            wp.trajectory.push_back(Position2D(p.lon, p.lat,p.yaw) );
            wpts->push_back(wp);
        }else{// cant parse file
            std::cout<<"cant parse file, check if format is four doubles and two ints in a row seperated with space"<<std::endl;
            return false;
        }
        counter++;
    }
    std::cout<<"Parsed "<<counter<<" lines"<<std::endl;

}

bool WaypointsFileSaver::savePoints(std::vector<Waypoint> wpts,std::string fileName)
{
    std::ofstream of(fileName+".pts");
    if (!of.is_open()) {
        std::cout << "failed to open " << fileName << "for writing"<<std::endl;
        return false;
    }
    std::stringstream ss;
    ss<<"lon,lat,yaw,dwellTimeSec,orientToYaw,trigerrOutput"<<std::endl;
    ss<<std::setprecision(10);
    for (int i = 0; i < wpts.size(); ++i) {
        int last = wpts.at(i).trajectory.size()-1;
        ss<<wpts.at(i).trajectory.at(last).x<<" "<<wpts.at(i).trajectory.at(last).y<<" "<<wpts.at(i).trajectory.at(last).yaw<<" "<<wpts.at(i).dwellTimeSec<<" "<<wpts.at(i).orientToYaw<<" "<<wpts.at(i).triggerOutputPin<<std::endl;
    }
    of<<ss.rdbuf();
    of.close();
    return true;
}

bool WaypointsFileSaver::saveAddedPoints(std::string fileName)
{
   return savePoints(waypointsToSave,fileName);

}

std::vector<std::string> WaypointsFileSaver::readFileNames()
{
    std::string srcDir = ".";

            DIR *dir;
    struct dirent *ent;
    std::vector<std::string> fileNames;
    if ((dir = opendir (srcDir.c_str())) != NULL) {
        /* print all the files and directories within directory */
        while ((ent = readdir (dir)) != NULL) {
            //  printf ("%s\n", ent->d_name);
            std::string s(ent->d_name);
            if(s.find(".pts")!=std::string::npos) //filter by extension
                fileNames.push_back(ent->d_name);
        }
        closedir (dir);
    } else {
        /* could not open directory */
        perror ("[WFS] could not open directory");

    }

    return fileNames;

}

bool WaypointsFileSaver::closeFile()
{
    getCurrentFileSize(); //writes file size in fileSize field for possible later access
    myfile.close();
    return (myfile.fail());

}

