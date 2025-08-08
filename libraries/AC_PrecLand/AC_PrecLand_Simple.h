#pragma once

#include "AC_PrecLand_config.h"

#if AC_PRECLAND_SIMPLE_ENABLED

#include <AC_PrecLand/AC_PrecLand_Backend.h>
#include <AP_Math/AP_Math.h>

/*
 * AC_PrecLand_Simple- implements precision landing simply
 */

class AC_PrecLand_Simple : public AC_PrecLand_Backend
{
public:

    // Constructor
    using AC_PrecLand_Backend::AC_PrecLand_Backend;

    // perform any required initialisation of backend
    void init() override;

    // retrieve updates from sensor
    void update() override;

private:
    Vector3p _dock_pos{0.0f, 0.0f, 0.0f};
    bool _dock_pos_valid{false};

};

#endif // AC_PRECLAND_IRLOCK_ENABLED
