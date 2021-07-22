#pragma once

#include <string>
#include <vector>

class NSParser
{
public:
    using TopicName = std::string;
    using Segment = std::string;

public:
    NSParser(const TopicName& topicName);
    NSParser(const NSParser&) = delete;
    NSParser& operator=(const NSParser&) = delete;
    ~NSParser() = default;

    auto begin() { return m_segments.begin(); }
    auto end() { return m_segments.end(); }

private:
    const TopicName m_topicName;
    std::vector<Segment> m_segments;
};
