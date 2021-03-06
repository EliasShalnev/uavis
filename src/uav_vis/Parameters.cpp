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
        if( std::string(argv[i]) == std::string("-u") ||
            std::string(argv[i]) == std::string("--uavModel") )
        {
            m_uavModelName = std::string( argv[i+1] );
        }
        else if( std::string(argv[i]) == std::string("-p") ||
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
            auto firstKindErrDouble = std::atof( firstKindErrStr.c_str() );
            m_firstKindError = static_cast<float>(firstKindErrDouble);
        }
        else if( std::string(argv[i]) == std::string("-s") || 
                 std::string(argv[i]) == std::string("--second-kind-error") )
        {
            auto secondKindErrStr = std::string( argv[i+1] );
            auto secondKindErrDouble = std::atof( secondKindErrStr.c_str() );
            m_secondKindError = static_cast<float>(secondKindErrDouble);
        }
    }
}


void Parameters::print() 
{
    ROS_INFO_STREAM("--uavModel " << m_uavModelName);
    ROS_INFO_STREAM("--port " <<  m_cameraPort);
    ROS_INFO_STREAM("--frame-proc-time " << static_cast<unsigned int>(m_frameProcessingTime) );
    ROS_INFO_STREAM("--second-kind-error " << m_secondKindError);
    ROS_INFO_STREAM("--first-kind-error " << m_firstKindError);
}
