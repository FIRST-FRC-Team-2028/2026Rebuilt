// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.VPose2d;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAndDrive extends Command {
  private SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  double smoothedXSpeed, smoothedYSpeed, smoothedTurningSpeed, xSpeed, ySpeed;
  boolean robotOrient = false;
  ChassisSpeeds chassisSpeeds;
  private final Joystick driverJoytick;
  private final Drivetrain drive;
  private Optional<Alliance> alliance;
  private PIDController controller;
  double targetTheta, kP=1./10., kI, kD, theta, turningSpeed, deadband = 2;
  VPose2d diff;
  /** Creates a new AimAndDrive. */
  public AimAndDrive(Drivetrain drive, Optional<Alliance> alliance, Joystick driverJoytick) {
    this.drive = drive;
    this.alliance = alliance;
    this.driverJoytick = driverJoytick;
    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller = new PIDController(kP, kI, kD);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xSpeed = xLimiter.calculate(MathUtil.applyDeadband(-driverJoytick.getRawAxis(OIConstants.kDriverXAxis), OIConstants.kDeadband))*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond; // Negative values go forward
    ySpeed = yLimiter.calculate(MathUtil.applyDeadband(-driverJoytick.getRawAxis(OIConstants.kDriverYAxis), OIConstants.kDeadband))*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    xSpeed *= 1. - (DriveConstants.kFineControlSpeed * driverJoytick.getRawAxis(OIConstants.kFineControlAxis))
                    + (DriveConstants.kFasterSpeed * driverJoytick.getRawAxis(OIConstants.kFastControlAxis));
    ySpeed *= 1. - (DriveConstants.kFineControlSpeed * driverJoytick.getRawAxis(OIConstants.kFineControlAxis))
                    + (DriveConstants.kFasterSpeed * driverJoytick.getRawAxis(OIConstants.kFastControlAxis));

    smoothedXSpeed = smoothedXSpeed + (xSpeed - smoothedXSpeed) * .18;
    smoothedYSpeed = smoothedYSpeed + (ySpeed - smoothedYSpeed) * .18;
    xSpeed = smoothedXSpeed;
    ySpeed = smoothedYSpeed;
    
    diff = drive.getVecToHub(alliance);
    targetTheta = Units.radiansToDegrees(Math.atan(diff.Y()/diff.X()));
    theta = drive.getPoseEstimatorPose().getRotation().getDegrees();
    turningSpeed = controller.calculate(theta, targetTheta);
    
    if(driverJoytick.getRawButtonPressed(OIConstants.kDriverRobotOrientedButton)) robotOrient=!robotOrient;
    if (!robotOrient) { //normal use
    // Relative to field
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                   xSpeed, ySpeed, turningSpeed, drive.getRotation2d());
    } else {
    // Relative to robot
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }
    drive.drive(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
