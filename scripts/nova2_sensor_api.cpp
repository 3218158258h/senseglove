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
 *  nova2_send_normalized_data(right_hand)
 *      Commands the glove to start streaming normalised sensor data.
 *      Call once at startup for each hand before reading sensor values.
 *      Returns 1 on success, 0 if the glove is not found.
 *
 *  nova2_get_normalized_input(right_hand, out_values, out_len)
 *      Fills *out_values (caller-allocated float[6]) with the latest
 *      normalised sensor values for the requested hand.
 *      Returns 1 on success, 0 if the glove is not found / not ready.
 *
 *  nova2_get_normalization_state(right_hand)
 *      Returns the current ENormalizationState as an int:
 *        -1 = glove not found
 *        -2 = sensor data unavailable
 *         1 = Unknown
 *         2 = SendingRawData
 *         3 = Normalizing_MoveFingers
 *         4 = Normalizing_AwaitConfirm
 *         5 = NormalizationFinished
 *
 *  nova2_get_raw_sensor_data(right_hand, out_values, out_len)
 *      Fills *out_values with 6 raw sensor readings directly from
 *      Nova2GloveSensorData::GetSensorValue (bypasses normalization).
 *      Useful to confirm the glove hardware is live even when
 *      GetNormalizedInput returns static/default values.
 *      Returns 1 on success, 0 on failure.
 *
 * Sensor layout (6 values, range 0.0 – 1.0 when normalised):
 *   [0]  Thumb   Abduction       (内收/外展)
 *   [1]  Thumb   FlexionProximal (屈伸)
 *   [2]  Index   FlexionProximal
 *   [3]  Index   FlexionDistal
 *   [4]  Middle  Flexion
 *   [5]  Ring    Flexion         (also used for Pinky)
 */

#include "Nova2Glove.hpp"
#include "Nova2GloveSensorData.hpp"
#include "DeviceList.hpp"
#include "Fingers.hpp"

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
 * Command the glove to start streaming normalised sensor data.
 * Call once per hand at startup before reading sensor values.
 *
 * @param right_hand  1 = right glove, 0 = left glove
 * @return            1 on success, 0 if the glove is not found
 */
int nova2_send_normalized_data(int right_hand)
{
    SGCore::Nova::Nova2Glove glove;
    const bool bRight = (right_hand != 0);
    if (!SGCore::Nova::Nova2Glove::GetNova2Glove(bRight, glove))
        return 0;
    return glove.SendNormalizedData() ? 1 : 0;
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

/**
 * Return the current sensor normalization state for one hand.
 *
 * ENormalizationState values (Nova2GloveSensorData.hpp):
 *   Unknown = 1, SendingRawData = 2, Normalizing_MoveFingers = 3,
 *   Normalizing_AwaitConfirm = 4, NormalizationFinished = 5
 *
 * @param right_hand  1 = right glove, 0 = left glove
 * @return  state as int; -1 if glove not found; -2 if sensor data unavailable
 */
int nova2_get_normalization_state(int right_hand)
{
    SGCore::Nova::Nova2Glove glove;
    const bool bRight = (right_hand != 0);
    if (!SGCore::Nova::Nova2Glove::GetNova2Glove(bRight, glove))
        return -1;

    SGCore::Nova::Nova2GloveSensorData sensorData;
    if (!glove.GetSensorData(sensorData))
        return -2;

    return static_cast<int>(sensorData.GetSensorState());
}

/**
 * Retrieve raw sensor values directly from Nova2GloveSensorData::GetSensorValue,
 * bypassing the SDK's normalization step.  Useful for diagnosing whether the
 * glove hardware is live even when GetNormalizedInput returns static values.
 *
 * Layout (indices match nova2_get_normalized_input):
 *   [0]  Thumb   Abduction
 *   [1]  Thumb   FlexionProximal
 *   [2]  Index   FlexionProximal
 *   [3]  Index   FlexionDistal
 *   [4]  Middle  FlexionProximal
 *   [5]  Ring    FlexionProximal
 *
 * @param right_hand  1 = right glove, 0 = left glove
 * @param out_values  Caller-allocated buffer of at least 6 floats
 * @param out_len     Written with 6 on success.  May be NULL.
 * @return            1 on success, 0 on failure
 */
int nova2_get_raw_sensor_data(int right_hand, float* out_values, int* out_len)
{
    if (!out_values)
        return 0;

    SGCore::Nova::Nova2Glove glove;
    const bool bRight = (right_hand != 0);
    if (!SGCore::Nova::Nova2Glove::GetNova2Glove(bRight, glove))
        return 0;

    SGCore::Nova::Nova2GloveSensorData sensorData;
    if (!glove.GetSensorData(sensorData))
        return 0;

    using F = SGCore::EFinger;
    using L = SGCore::Nova::Nova2GloveSensorData::ESensorLocation;

    out_values[0] = sensorData.GetSensorValue(F::Thumb,  L::Abduction);
    out_values[1] = sensorData.GetSensorValue(F::Thumb,  L::FlexionProximal);
    out_values[2] = sensorData.GetSensorValue(F::Index,  L::FlexionProximal);
    out_values[3] = sensorData.GetSensorValue(F::Index,  L::FlexionDistal);
    out_values[4] = sensorData.GetSensorValue(F::Middle, L::FlexionProximal);
    out_values[5] = sensorData.GetSensorValue(F::Ring,   L::FlexionProximal);

    if (out_len)
        *out_len = 6;

    return 1;
}

} // extern "C"
