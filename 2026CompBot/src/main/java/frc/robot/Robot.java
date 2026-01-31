// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Climber;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  Joystick driverJoytick, mechJoytick1, mechJoytick2;
  public Robot() {
    m_robotContainer = new RobotContainer();
    this.driverJoytick = m_robotContainer.getDriverJoystick();
    this.mechJoytick1 = m_robotContainer.getMech1Joystick();
    this.mechJoytick2 = m_robotContainer.getMech2Joystick();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
  }

  @Override
  public void testPeriodic() {
    if (Constants.CLIMBER_AVAILABLE){
      if (driverJoytick.getRawButtonPressed(1)){
        m_robotContainer.getClimber().switchSoftLimits(false, false);
      }
      if (driverJoytick.getRawButtonPressed(2)){
        m_robotContainer.getClimber().switchSoftLimits(true, true);
      }
    }
  }

  @Override
  public void testExit() {}
}
