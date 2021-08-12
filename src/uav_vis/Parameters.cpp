#include "uav_vis/Parameters.h"

#include <ros/io.h>

Parameters* Parameters::m_instance = nullptr;


Parameters::~Parameters() 
{
    if(m_instance == nullptr) { delete m_instance; }
}


Parameters* Parameters::getInstance() 
{
    if(m_instance == nullptr) { m_instance = new Parameters(); }
    return m_instance;
}

void Parameters::parseArgs(int argc, char** argv) 
{
    for(int i=1; i+1<argc; i+=2)
    {
        if( std::string(argv[i]) == std::string("-p") ||
            std::string(argv[i]) == std::string("--port") ) 
        { 
            auto portStr = std::string( argv[i+1] );
            auto portInt = std::stoi(portStr);
            m_cameraPort = static_cast<uint16_t>(portInt);
        }
        else if( std::string(argv[i]) == std::string("-t") || 
                 std::string(argv[i]) == std::string("--frame-proc-time") )
        {
            auto frameProcessingTimeStr = std::string( argv[i+1] );
            auto frameProcessingTimeInt = std::stoi(frameProcessingTimeStr);
            m_frameProcessingTime = static_cast<uint8_t>(frameProcessingTimeInt);
        }
        else if( std::string(argv[i]) == std::string("-f") || 
                 std::string(argv[i]) == std::string("--first-kind-error") )
        {
            auto firstKindErrStr = std::string( argv[i+1] );
            auto firstKindErrInt = std::stoi(firstKindErrStr);
            m_firstKindError = static_cast<uint8_t>(firstKindErrInt);
        }
        else if( std::string(argv[i]) == std::string("-s") || 
                 std::string(argv[i]) == std::string("--second-kind-error") )
        {
            auto secondKindErrStr = std::string( argv[i+1] );
            auto secondKindErrInt = std::stoi(secondKindErrStr);
            m_secondKindError = static_cast<uint8_t>(secondKindErrInt);
        }
    }
}


void Parameters::print() 
{
    ROS_INFO_STREAM("--port " <<  m_cameraPort);
    ROS_INFO_STREAM("--frame-proc-time " << static_cast<unsigned int>(m_frameProcessingTime) );
    ROS_INFO_STREAM("--second-kind-error " << static_cast<unsigned int>(m_secondKindError) );
    ROS_INFO_STREAM("--first-kind-error " << static_cast<unsigned int>(m_firstKindError) );
    
}
