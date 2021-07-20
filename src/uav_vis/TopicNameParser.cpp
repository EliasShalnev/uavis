#include "uav_vis/TopicNameParser.h"

#include <ros/console.h>


TopicNameParser::TopicNameParser(const TopicName& topicName) 
    : m_topicName(topicName)
{
    std::size_t segmentBegin = m_topicName.find('/');
    std::size_t segmentEnd = m_topicName.find('/');
     
    while(segmentEnd != std::string::npos)
    {
        segmentEnd = m_topicName.find('/', segmentEnd+1);
        
        Segment segment = m_topicName.substr(segmentBegin, segmentEnd-segmentBegin);

        m_segments.emplace_back(segment);

        segmentBegin = segmentEnd;
    }
}
