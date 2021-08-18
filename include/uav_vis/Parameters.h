#pragma once

#include <stdint.h>
#include <string>

class Parameters 
{
public:
    Parameters(const Parameters&) = delete;
    Parameters& operator=(const Parameters&) = delete;
    ~Parameters();
    
    static Parameters* getInstance();

    void parseArgs(int argc, char** argv);

    void print();

    const uint16_t& getCameraPort() const { return m_cameraPort; }
    const uint8_t& getFrameProcessingTime() const { return m_frameProcessingTime; } 
    const float& getFirstKindError() const { return m_firstKindError; }
    const float& getSecondKindError() const { return m_secondKindError; }

private:
    uint16_t m_cameraPort = 5600;
    uint8_t m_frameProcessingTime = 3;
    float m_firstKindError = 15;
    float m_secondKindError = 75;

private:
    Parameters() = default;
    static Parameters* m_instance;
};
