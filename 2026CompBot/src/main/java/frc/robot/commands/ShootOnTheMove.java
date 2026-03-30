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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.VPose2d;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.util.HubTracker;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootOnTheMove extends Command {
  private SlewRateLimiter xLimiter, yLimiter;
  double smoothedXSpeed, smoothedYSpeed, xSpeed, ySpeed;
  boolean robotOrient = false;
  
  private final Joystick driverJoytick;
  private final Drivetrain drive;
  private Optional<Alliance> alliance;
  private PIDController controller;
  double targetTheta, kP=1.8/10., kI, kD, theta, turningSpeed, deadband = 2., offsetY, robotXspeed, robotYspeed;
  VPose2d diff, velVec;
  private final Shooter shooter;
  double shootSpeedDeadband = 100;
  boolean shooting = false;
  double velocityIncrease = 200;
  double velocity; 
  Timer timer;
  /** Creates a new ShootOnTheMove */
  public ShootOnTheMove(Drivetrain drive, Optional<Alliance> alliance, Joystick driverJoytick, Shooter shooter) {
    this.drive = drive;
    this.alliance = alliance;
    this.driverJoytick = driverJoytick;
    this.shooter = shooter;
    timer = new Timer();
    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    controller = new PIDController(kP, kI, kD);

    addRequirements(drive, shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    diff = drive.getVecToHub(alliance);
    velocity = shooter.shiftRPM(shooter.shooterRPM(diff.norm()), robotXspeed);
    shooter.setShooterSpeed(velocity+2*velocityIncrease);
    shooting = false;
    xSpeed=0;
    ySpeed=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Joystick stuff for driving
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
    theta = drive.getPoseEstimatorPose().getRotation().getDegrees();
    velVec = new VPose2d(xSpeed, ySpeed, 0).rotateby(Units.degreesToRadians(theta));
    robotXspeed = velVec.X();
    robotYspeed=velVec.Y();
    //Turning to face the hub
    offsetY = robotYspeed*shooter.ballAirTime(velocity)*0.32; //Accounts for movement left and right of the hub (Y direction)
    diff = drive.getVecToHub(alliance)/*.rotateby(theta)*/;
    targetTheta = Units.radiansToDegrees(Math.atan((diff.Y()-offsetY)/diff.X()));
    turningSpeed = (controller.calculate(theta, targetTheta));
    SmartDashboard.putNumber("Turning Speed", turningSpeed);
    SmartDashboard.putNumber("TargetThera", targetTheta);
    SmartDashboard.putNumber("Theta", theta);
    
    if(driverJoytick.getRawButtonPressed(OIConstants.kDriverRobotOrientedButton)) robotOrient=!robotOrient;
   ChassisSpeeds chassisSpeeds;
    if (!robotOrient) { //normal use
    // Relative to field
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                   xSpeed, ySpeed, turningSpeed, drive.getPoseEstimatorPose().getRotation());
    } else {
    // Relative to robot
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }
    drive.drive(chassisSpeeds);

    //Shooter Speed
    velocity = shooter.shiftRPM(shooter.shooterRPM(diff.norm()), robotXspeed);  //Accounts for movement to and away from the hub (X direction)
    System.out.println(velocity);
    //velocity = shooter.shooterRPM(diff.norm());
    if (!shooting){
      shooter.setShooterSpeed(velocity+2*velocityIncrease);
    if(shooter.getShooterVelocity()> velocity+2*velocityIncrease-shootSpeedDeadband){
      timer.start();
      shooter.setConveyorSpeed(ShooterConstants.conveyorShootSpeed);
      shooting = true;
      }
    }
    if (timer.hasElapsed(.75))shooter.setShooterSpeed(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (HubTracker.getCurrentShift().isPresent()){
      if (HubTracker.isActive()){
        shooter.setShooterSpeed(ShooterConstants.OptimalShootSpeed+2.1*velocityIncrease);
        shooter.setConveyorSpeed(0);
      } else  shooter.stopShooting();
    } else shooter.stopShooting();

    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
