#include "logfilesaver.hpp"
#include <sstream>
#include <ctime>
#include <iomanip>

LogFileSaver::LogFileSaver()
{
 // openFile();
//writeHeader();

}


LogFileSaver LogFileSaver::logfilesaver;

void LogFileSaver::writeString( std::stringstream & entry)
{
if(myfile.is_open())
    myfile << entry.rdbuf()<<std::endl;

}


void LogFileSaver::writeEntry(LogEntry entry)
{

    myfile << entry.ts<<"\t"<< entry.dist<<"\t"<< entry.angle<<"\t"<< (int)entry.lightBumps<<"\n";

}

void LogFileSaver::openFile()
{
    string filename;
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream ss;
    ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S.vsmlog");
    filename = ss.str();
       myfile= ofstream(filename);
    //myfile = std::fstream(filename, std::ios::out);// | std::ios::binary);
    this->fileName =filename;

}

void LogFileSaver::writeHeader()
{
    myfile<< "time\tdist\tangle\tlightbumps\n";

}

bool LogFileSaver::closeFile()
{
    getCurrentFileSize(); //writes file size in fileSize field for possible later access
    myfile.close();
    return (myfile.fail());

}

