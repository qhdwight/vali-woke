#pragma once

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

#define DRIVE_STICK_PORT 0
#define SLIDE_STICK_PORT 1

namespace ctr {
    using ctre::phoenix::motorcontrol::can::TalonSRX;
    using ctre::phoenix::motorcontrol::ControlMode;
}

class Robot : public frc::TimedRobot {
private:
    frc::Joystick m_DriveStick{DRIVE_STICK_PORT}, m_TurnStick{SLIDE_STICK_PORT};
    ctr::TalonSRX
            m_LeftMaster{1}, m_LeftSlaveOne{2}, m_LeftSlaveTwo{3},
            m_RightMaster{6}, m_RightSlaveOne{5}, m_RightSlaveTwo{4},
            m_SliderMaster{10};
    double m_LastWheel, m_NegativeInertiaAccumulator, m_QuickStopAccumulator;

    static double handleDeadBand(double input);

    static double clamp01(double input);
public:
    void RobotInit() override;

    void DisabledInit() override;

    void AutonomousInit() override;

    void AutonomousPeriodic() override;

    void TeleopInit() override;

    void TeleopPeriodic() override;

    void TestInit() override;

    void TestPeriodic() override;
};
