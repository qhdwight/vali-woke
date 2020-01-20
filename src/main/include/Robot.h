#pragma once

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

#define DRIVE_STICK_PORT 0
#define SLIDE_STICK_PORT 1

namespace ctr {
    using ctre::phoenix::motorcontrol::ControlMode;
    using ctre::phoenix::motorcontrol::can::TalonSRX;
    using ctre::phoenix::motorcontrol::can::VictorSPX;
}

class Robot : public frc::TimedRobot {
private:
    frc::Joystick m_DriveStick{DRIVE_STICK_PORT}, m_TurnStick{SLIDE_STICK_PORT};
    // Vali
    ctr::TalonSRX
            m_LeftMaster{1}, m_LeftSlaveOne{2}, m_LeftSlaveTwo{3},
            m_RightMaster{6}, m_RightSlaveOne{5}, m_RightSlaveTwo{4};
    // Switch-bot
//    ctr::TalonSRX m_LeftMaster{16}, m_RightMaster{15};
//    ctr::VictorSPX
//            m_LeftSlaveOne{1}, m_LeftSlaveTwo{2},
//            m_RightSlaveOne{13}, m_RightSlaveTwo{14};
    double m_LastWheel, m_NegativeInertiaAccumulator, m_QuickStopAccumulator;

    static double handleDeadBand(double input);

    static double clamp01(double input);

public:
    void RobotInit() override;

    void RobotPeriodic() override;

    void DisabledInit() override;

    void AutonomousInit() override;

    void AutonomousPeriodic() override;

    void TeleopInit() override;

    void TeleopPeriodic() override;

    void TestInit() override;

    void TestPeriodic() override;
};
