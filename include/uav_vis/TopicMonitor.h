#pragma once

#include <functional>
#include <vector>

#include <ros/node_handle.h>
#include <ros/timer.h>
#include <ros/master.h>


class TopicMonitor
{
public:
    using RequestCallback =  std::function<void(const XmlRpc::XmlRpcValue&)>;

public:
    TopicMonitor();
    TopicMonitor(const TopicMonitor&) = delete;
    TopicMonitor& operator=(const TopicMonitor&) = delete;
    ~TopicMonitor();

protected:
    virtual void systemStateRequest();

    void onPublishedTopics(const RequestCallback& callback);
    void onSubscribedTopics(const RequestCallback& callback);

private:
    void timerCallback(const ros::TimerEvent &event);

private:
    ros::NodeHandle m_nodeHandle;
    ros::Timer m_requestTimer;

    std::vector<RequestCallback> m_onPublishedTopics;
    std::vector<RequestCallback> m_onSubscribedTopics;

    const std::string m_getSystemState = "getSystemState";
};

