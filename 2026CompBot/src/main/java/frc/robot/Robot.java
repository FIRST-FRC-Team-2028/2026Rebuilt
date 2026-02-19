// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;

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
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

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
    m_robotContainer.turnOnDrive();
  }

  boolean hubActive = true;
  int delay = 1;
  boolean dataSent = false;
  @Override
  public void teleopPeriodic() {
    if (!dataSent){
     String gameData = DriverStation.getGameSpecificMessage();
     if (!gameData.isEmpty()) {
      // if data is R and we are Red, we score in the first period
      if (gameData.charAt(0)=='R' && m_robotContainer.getAlliance().get()==Alliance.Red
       ||gameData.charAt(0)=='B' && m_robotContainer.getAlliance().get()==Alliance.Blue) 
         delay = 0;
      dataSent = true;
     }
    }
    double matchTime = DriverStation.getMatchTime();
    if (matchTime > 130) {
    // Transition shift, hub is active.
      hubActive = true;
    } else if (matchTime > 105) {
    // Shift 1
      hubActive = (1+delay)%2 ==1;
    } else if (matchTime > 80) {
    // Shift 2
      hubActive = (1+delay)%2 ==0;
    } else if (matchTime > 55) {
    // Shift 3
      hubActive = (1+delay)%2 ==1;
    } else if (matchTime > 30) {
    // Shift 4
      hubActive = (1+delay)%2 ==0;
    } else {
    // End game, hub always active.
      hubActive = true;
    } 
    SmartDashboard.putBoolean("HubActive", hubActive);
  }

  @Override
  public void teleopExit() {}

  boolean testClimb, testIntakeRoller, testIntakeJoint, testConveyor,testShoot,vbus;
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    m_robotContainer.turnOffDrive();
    testClimb=false;
    testIntakeRoller=false;
    testIntakeJoint=false;
    testConveyor=false;
    testShoot=false;
    SmartDashboard.putNumber("testP",0.);
    vbus = true;  // switch between vbus and closed loop
  }

  String onstring;
  double testVal=0.;
  double testVal2 = 0.;
  @Override
  public void testPeriodic() {
    if(mechJoytick2.getRawButtonPressed(Constants.OIConstants.TEST_CLMODE)) vbus=!vbus;
    SmartDashboard.putBoolean("vbus",vbus);
    if(mechJoytick1.getRawButtonPressed(Constants.OIConstants.TEST_INTAKE_JOINT)){
      testIntakeJoint=!testIntakeJoint;
      if(testIntakeJoint)SmartDashboard.putNumber("testP",Constants.IntakeConstants.jointP);
      // Use typein to play with kP for the joint controllers
    }
    if(mechJoytick1.getRawButtonPressed(Constants.OIConstants.TEST_INTAKE_ROLLER)){
      testIntakeRoller=!testIntakeRoller;
    }
    if(mechJoytick1.getRawButtonPressed(Constants.OIConstants.TEST_CONVEYOR)){
      testConveyor=!testConveyor;
    }
    if(mechJoytick1.getRawButtonPressed(Constants.OIConstants.TEST_SHOOT)){
      testShoot=!testShoot;
      if(testShoot)SmartDashboard.putNumber("testP",Constants.ShooterConstants.shooterShootSpeed); 
      // Use typein to play with Shooter RPM
    }
    if(mechJoytick1.getRawButtonPressed(Constants.OIConstants.TEST_CLIMB)){
      testClimb=!testClimb;
    }
    onstring="";
    testVal=0.;
    testVal2 = 0.;
    if (testIntakeJoint) onstring+=" Joint ";
    if (testClimb)       onstring+=" Climb ";
    if (testConveyor)    onstring+=" Conveyor ";
    if (testIntakeRoller)onstring+=" Roller ";
    if (testShoot)       onstring+=" Shoot ";
    SmartDashboard.putString("testMode",onstring);

    if (Constants.CLIMBER_AVAILABLE){
      if(testIntakeJoint){
        if (mechJoytick1.getRawButtonPressed(OIConstants.SOFTLIMIT_DISABLE)){
          m_robotContainer.getClimber().switchSoftLimits(false, false);
        }
        if (mechJoytick1.getRawButtonPressed(OIConstants.SOFTLIMIT_ENABLE)){
          m_robotContainer.getClimber().switchSoftLimits(true, true);
        }
        if(vbus)m_robotContainer.getClimber().setClimberSpeed(driverJoytick.getRawAxis(OIConstants.RIGHTSTICKVERT)*.3);
        testVal = m_robotContainer.getClimber().getClimberPosition();
        if (mechJoytick1.getRawButtonPressed(OIConstants.RESETENCODER)){
          m_robotContainer.getClimber().switchSoftLimits(true, true);  
        // yeah there is already a special button for this, but this is consistent with other functionality and doesn't hurt anything
        }
      }
    }
    if (Constants.INTAKE_AVAILABLE){
      if(testIntakeJoint){
        if(vbus)m_robotContainer.getIntake().goJoint(driverJoytick.getRawAxis(OIConstants.RIGHTSTICKVERT)*.3);
        testVal = m_robotContainer.getIntake().getJointPosition();
        testVal2 = m_robotContainer.getIntake().getJointPosition2();
        if (mechJoytick1.getRawButtonPressed(OIConstants.SOFTLIMIT_DISABLE)){
          m_robotContainer.getIntake().switchSoftLimits(false, false);
        }
        if (mechJoytick1.getRawButtonPressed(OIConstants.SOFTLIMIT_ENABLE)){
          m_robotContainer.getIntake().switchSoftLimits(true, true);
        }
        if (mechJoytick1.getRawButtonPressed(OIConstants.RESETENCODER)){
          m_robotContainer.getIntake().resetJointEncoder();
        }
        if (mechJoytick1.getRawButtonPressed(OIConstants.TEST_LOW_CONTROL)){
          m_robotContainer.getIntake().setJointPosition(IntakeConstants.JointUpPosition);
          m_robotContainer.getIntake().setJointIdleMode(IdleMode.kBrake);
        }
        if (mechJoytick1.getRawButtonPressed(OIConstants.TEST_HIGH_CONTROL)){
          m_robotContainer.getIntake().setJointPosition(IntakeConstants.JointPickupPosition);
          m_robotContainer.getIntake().setJointIdleMode(IdleMode.kCoast);
        }
        double testP = SmartDashboard.getNumber("testP",0.);
        m_robotContainer.getIntake().setJointPID(testP,0.,0.);
     }
     if(testIntakeRoller){
        if(vbus)m_robotContainer.getIntake().rollers(driverJoytick.getRawAxis(OIConstants.RIGHTSTICKVERT));
        testVal = m_robotContainer.getIntake().getRollerSpeed();
     }
    }
    if (Constants.SHOOTER_AVAILABLE){
       if(testConveyor){
        //if(vbus)m_robotContainer.getShoot().goConvey(driverJoytick.getRawAxis(OIConstants.RIGHTSTICKVERT));
        //testVal = m_robotContainer.getShoot().getConveySpeed();
      }
      if(testShoot){
        if(vbus)m_robotContainer.getShoot().setShooterVbus(driverJoytick.getRawAxis(OIConstants.RIGHTSTICKVERT));
        if (mechJoytick1.getRawButtonPressed(OIConstants.TEST_LOW_CONTROL)){
          m_robotContainer.getShoot().setShooterSpeed(ShooterConstants.shooterShootSpeed);
        }
        if (mechJoytick1.getRawButtonPressed(OIConstants.TEST_HIGH_CONTROL)){
          m_robotContainer.getShoot().setShooterSpeed(SmartDashboard.getNumber("testP",0.));
        }
        testVal = m_robotContainer.getShoot().getShooterVelocity();
      }
    }
    if (Constants.PIXYCAM_AVAILABLE){

    }
    SmartDashboard.putNumber("TestVal",testVal);
    SmartDashboard.putNumber("TestVal2",testVal2);
  }

  @Override
  public void testExit() {}
}
