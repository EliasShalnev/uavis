#pragma once

#include <string>
#include <vector>

class TopicNameParser
{
public:
    using TopicName = std::string;
    using Segment = std::string;

public:
    TopicNameParser(const TopicName& topicName);
    TopicNameParser(const TopicNameParser&) = delete;
    TopicNameParser& operator=(const TopicNameParser&) = delete;
    ~TopicNameParser() = default;

    auto begin() { return m_segments.begin(); }
    auto end() { return m_segments.end(); }

private:
    const TopicName m_topicName;
    std::vector<Segment> m_segments;
};
