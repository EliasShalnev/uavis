#include "uav_vis/Target.h"


Target::Target(const ModelName& modelName)
    : Model(modelName)
    , m_damageSub(m_nh, "/" + modelName + "/" + "target_damage", 10)
{ }



bool Target::isActive() const 
{
    auto damage = m_damageSub.getMessage()->damage;
    if(damage < 1) { return true; }
    return false;
}
