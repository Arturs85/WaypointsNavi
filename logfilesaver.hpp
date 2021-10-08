#ifndef LOGFILESAVER_HPP
#define LOGFILESAVER_HPP

#include <vector>
#include <fstream>
#include <string.h>

using namespace std;
typedef struct LogEntry{int32_t ts; int16_t dist; int16_t angle; uint8_t lightBumps;
                       LogEntry(int32_t ts1, int16_t dist1, int16_t angle1, uint8_t lightBumps1):
                           ts(ts1),dist(dist1),angle(angle1),lightBumps(lightBumps1){}
                       }LogEntry;

class LogFileSaver
{


public:
    LogFileSaver();

static LogFileSaver logfilesaver;
    ofstream myfile;
    size_t fileSize;
    std::string fileName;


    void writeEntry(LogEntry entry);
    void openFile();
    void writeHeader();

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

    void writeString(stringstream &entry);
};

#endif // LOGFILESAVER_HPP

