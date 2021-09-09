#pragma once

#include "target_dmg/TargetDamage.h"

#include "uav_vis/Model.h"
#include "uav_vis/SubMonitor.h"

class Target : public Model
{
public:
    Target(const ModelName& modelName);
    Target(const Target&) = delete;
    Target& operator=(const Target&) = delete;
    ~Target() = default;

    bool isActive() const override;

private:
    SubMonitor<target_dmg::TargetDamage> m_damageSub;
};
