#include <array>
#include <cassert>

#include "agent_control_driver/axis_mapping.hpp"

int main()
{
    using agent_control_driver::make_sdk_axes;
    using agent_control_driver::reorder_external_axes;

    const auto sdk = make_sdk_axes(1.0f, 2.0f, 3.0f, 4.0f);
    assert(sdk[0] == 1.0f);
    assert(sdk[1] == 2.0f);
    assert(sdk[2] == 3.0f);
    assert(sdk[3] == 4.0f);

    // external [forward, yaw, strafe, head] -> SDK [forward, strafe, yaw, head]
    const auto reordered = reorder_external_axes(1.0f, 2.0f, 3.0f, 4.0f);
    assert(reordered[0] == 1.0f);
    assert(reordered[1] == 3.0f);
    assert(reordered[2] == 2.0f);
    assert(reordered[3] == 4.0f);
    return 0;
}
