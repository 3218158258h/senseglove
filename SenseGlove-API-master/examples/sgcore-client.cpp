
#include <iostream>
#include <thread>
#include <limits>
#include <signal.h>
#include <atomic>

#include <SenseGlove/Connect/SGConnect.hpp>
#include <SenseGlove/Core/Debugger.hpp>
#include <SenseGlove/Core/SenseCom.hpp>
#include <SenseGlove/Core/Library.hpp>

#include <SenseGlove/Core/HandLayer.hpp>

#include <SenseGlove/Core/HandPose.hpp>
#include <SenseGlove/Core/Quat.hpp>
#include <SenseGlove/Core/Vect3D.hpp>

#include <SenseGlove/Core/HapticGlove.hpp>
#include <SenseGlove/Core/Tracking.hpp>
#include <SenseGlove/Core/StringUtils.hpp>

using namespace SGCore;
using namespace SGCore::Kinematics;

// 原子停止标志位
static std::atomic<bool> stopFlag = false;
void signal_handler(int signal)
{
    std::cout << "程序已收到信号，开始退出" << std::endl;
    stopFlag = true;
}

///  暂停程序，等待用户输入
static void Pause()
{
    // std::cout << "按回车键继续...";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

// 校准手套
static void CalibrateGlove(bool rightHand)
{
    if (!HandLayer::DeviceConnected(rightHand))
        return;

    std::string hand = rightHand ? "右手" : "左手";
    SGCore::EHapticGloveCalibrationState calState = HandLayer::GetCalibrationState(rightHand);
    SGCore::HandPose handPose;
    if (calState == EHapticGloveCalibrationState::Unknown)
    {
        std::cout << hand << "的校准状态未知，无法获取有效的手部姿态数据..." << std::endl;
    }
    if (calState == EHapticGloveCalibrationState::MoveFingers) // 简易校准模式
    {
        int32_t timeout = 20;              // 每20毫秒更新一次，Nova手套刷新率不超过60Hz
        int32_t sanityTimeout = 20 * 1000; // 10秒超时时间
        int32_t sanityTimer = 0;

        int32_t moveTimer = 0;
        int32_t moveTimeout = 1 * 1000; // 开始移动后等待1秒
        std::cout << hand << "尚未完成校准。如果是Nova手套，调用GetHandPose()/GetSensorData()函数获取数据时会自动进行校准" << std::endl;
        std::cout << "请在" << sanityTimeout / 1000.0f << "秒内活动" << hand << "上的所有传感器，重点活动拇指传感器" << std::endl;
        do
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(timeout)); // 秒转毫秒
            HandLayer::GetHandPose(rightHand, handPose);                     // 通常需要多次获取姿态数据
            sanityTimer += timeout;
            calState = HandLayer::GetCalibrationState(rightHand);
            if (calState == EHapticGloveCalibrationState::AllSensorsMoved)
            {
                moveTimer += timeout;
            }
        } while (sanityTimer < sanityTimeout && moveTimer < moveTimeout);

        if (calState == EHapticGloveCalibrationState::MoveFingers) // 仍未完成足够的传感器移动，已达到最大超时时间
        {
            std::cout << "你尚未活动全部传感器，部分角度数据可能存在偏差，但仍可以获取手部姿态。" << std::endl;
        }
        else
        {
            std::cout << "手指活动完成！" << std::endl;
        }
    }
}

///  获取手部姿态数据
/// <param name="rightHand">是否为右手</param>
static void GetHandPose(bool rightHand)
{
    if (!HandLayer::DeviceConnected(rightHand))
        return;

    std::string hand = rightHand ? "右手" : "左手";
    HandPose handPose;
    // 实际获取手部姿态
    if (HandLayer::GetHandPose(rightHand, handPose))
    {
        std::cout << "成功获取" << hand << "的姿态数据：" << std::endl;
        std::cout << handPose.ToString() << std::endl;
    }
    else
    {
        std::cout << "无法获取" << hand << "的姿态数据，可能是传感器数据损坏或手套已断开连接，请稍后重试" << std::endl;
    }
}

// 打印版本信息
static void PrintVersion()
{
    std::cout << "V " << SGCore::Library::Version() << "，Compiled Backend：" << SGCore::Library::BackendVersion();
    if (SGCore::Library::GetBackendType() == SGCore::EBackendType::SharedMemory) // 默认情况下，库通过SGConnect使用共享内存通信
    {
        std::cout << "，Using SGConnect Version：" << SGCore::Library::SGConnectVersion(); // 如果替换了SGConnect.dll，这里会显示其当前版本
    }
    std::cout << std::endl;
}

// 确保设备连接
// 连接SenseGlove设备通过独立的连接进程实现，该进程包含在SGConnect库中
// 可以检测连接进程是否正在运行，通常该进程在SenseCom软件中运行
// 如果用户尚未启动该进程，建议强制启动
static void EnsureDeviceConnected()
{
    bool connectionsActive = SGCore::SenseCom::ScanningActive(); // 如果SenseCom已启动通信进程，返回true
    if (!connectionsActive)                                      // 如果进程未运行，可以强制启动SenseCom（前提是该软件至少在本机运行过一次）
    {
        std::cout << "SenseCom 未运行，无法连接SenseGlove设备" << std::endl;
        bool startedSenseCom = SGCore::SenseCom::StartupSenseCom(); // 成功启动进程返回true
        if (startedSenseCom)
        {
            std::cout << "成功启动SenseCom，设备连接需要等待几秒..." << std::endl;
            connectionsActive = SGCore::SenseCom::ScanningActive(); // 调用启动函数后会立即返回false，因为程序需要初始化时间
                                                                    //  即使SenseCom已启动，也无法保证用户已开启设备，后续会详细说明
        }
        else // 如果启动失败，可能是从未运行过SenseCom，或进程已在运行（此时ScanningActive应返回true）
        {
            std::cout << "无法启动SenseCom进程" << std::endl;
        }
    }
}

// 检测设备连接状态
static void CheckDeviceConnection()
{
    // 检测设备连接状态
    int32_t gloveAmount = HandLayer::GlovesConnected(); // 获取系统中已连接的手套数量
    while (gloveAmount == 0 && !stopFlag.load())        // 循环检测，直到连接到手套
    {
        std::cout << "未检测到力反馈手套，输入回车尝试重新连接..." << std::endl;
        std::cout << "如需退出，请按Ctrl+C，然后回车..." << std::endl;
        Pause();
        gloveAmount = HandLayer::GlovesConnected();
    }
    if (stopFlag.load())
    {
        exit(0); // 如果在等待过程中收到退出信号，安全退出程序
    }

    // 已成功连接至少一只手套
    if (gloveAmount == 1)
    {
        std::cout << "系统已连接1只力反馈手套" << std::endl;
        bool rightHand = HandLayer::GetFirstGloveHandedness();
        std::cout << "设备为：" << (rightHand ? "右手" : "左手") << "版，型号：" << SGDevice::ToString(HandLayer::GetDeviceType(rightHand)) << std::endl;
    }
    else
    {
        std::cout << "系统已连接" << std::to_string(gloveAmount) << "只力反馈手套" << std::endl;

        if (HandLayer::DeviceConnected(true))
        {
            std::cout << "右手设备型号：" << SGDevice::ToString(HandLayer::GetDeviceType(true)) << std::endl;
        }
        else
        {
            std::cout << "未连接右手设备" << std::endl;
        }

        if (HandLayer::DeviceConnected(false))
        {
            std::cout << "左手设备型号：" << SGDevice::ToString(HandLayer::GetDeviceType(false)) << std::endl;
        }
        else
        {
            std::cout << "未连接左手设备" << std::endl;
        }
    }
}

int32_t main()
{
    signal(SIGINT, signal_handler);  // Ctrl+C
    signal(SIGTERM, signal_handler); // kill
    // 设置调试等级，可根据需要开启
    // SGCore::Diagnostics::Debugger::SetDebugLevel(SGCore::Diagnostics::EDebugLevel::All);
    // 检查库文件信息
    // 显示库版本信息，寻求技术支持时非常有用
    PrintVersion();

    // 确保设备连接
    EnsureDeviceConnected();

    // 检测设备连接状态
    CheckDeviceConnection();

    // 校准手套
    // 如果设备已连接但未校准，获取手部姿态数据时会自动进行校准，但也可以在此处手动调用校准函数
    CalibrateGlove(true);  // 右手
    CalibrateGlove(false); // 左手

    while (!stopFlag.load()) // 主循环，持续发布手部姿态
    {
        GetHandPose(true);
        GetHandPose(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(15)); // 以15ms的间隔更新数据
    }
}
