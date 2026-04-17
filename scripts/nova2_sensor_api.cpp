/**
 * nova2_sensor_api.cpp
 *
 * Thin C-export wrapper around the SenseGlove C++ SDK.
 * Compile to a shared library (see Makefile) and load from Python via ctypes —
 * no ROS2 required.
 *
 * Exported symbols
 * ─────────────────
 *  nova2_sensecom_running()
 *      Returns 1 if the SenseCom companion process is running, 0 otherwise.
 *
 *  nova2_get_normalized_input(right_hand, out_values, out_len)
 *      Fills *out_values (caller-allocated float[6]) with the latest
 *      normalised sensor values for the requested hand.
 *      Returns 1 on success, 0 if the glove is not found / not ready.
 *
 * Sensor layout (6 values, range 0.0 – 1.0):
 *   [0]  Thumb   Abduction       (内收/外展)
 *   [1]  Thumb   FlexionProximal (屈伸)
 *   [2]  Index   FlexionProximal
 *   [3]  Index   FlexionDistal
 *   [4]  Middle  Flexion
 *   [5]  Ring    Flexion         (also used for Pinky)
 */

#include "Nova2Glove.hpp"
#include "DeviceList.hpp"

#include <cstring>
#include <vector>

extern "C"
{

/**
 * Returns 1 if SenseCom is running (prerequisite for any glove data).
 */
int nova2_sensecom_running()
{
    return SGCore::DeviceList::SenseComRunning() ? 1 : 0;
}

/**
 * Retrieve the latest normalised sensor values for one hand.
 *
 * @param right_hand  1 = right glove, 0 = left glove
 * @param out_values  Caller-allocated buffer of at least 6 floats
 * @param out_len     On success, written with the number of values placed in
 *                    out_values (always ≤ 6).  May be NULL.
 * @return            1 on success, 0 on failure
 */
int nova2_get_normalized_input(int right_hand, float* out_values, int* out_len)
{
    if (!out_values)
        return 0;

    SGCore::Nova::Nova2Glove glove;
    const bool bRight = (right_hand != 0);

    if (!SGCore::Nova::Nova2Glove::GetNova2Glove(bRight, glove))
        return 0;

    std::vector<float> vals;
    if (!glove.GetNormalizedInput(vals))
        return 0;

    const int count = static_cast<int>(vals.size() < 6 ? vals.size() : 6);
    std::memcpy(out_values, vals.data(), static_cast<std::size_t>(count) * sizeof(float));

    if (out_len)
        *out_len = count;

    return 1;
}

} // extern "C"
