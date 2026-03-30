// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.util.HubTracker;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AdvancedShoot extends Command {
  private final Shooter shooter;
  private final Drivetrain drive;
  double shootSpeedDeadband = 100;
  boolean shooting = false;
  double shotCount = 0;
  double velocityIncrease = 200;
  double velocity; 
  Timer timer;
  /** Controls process to shoot.
   * <p> Delays feeding until shooting wheels are up to speed.
   * <p> Computes Wheel Speed based on distance from Tower.
   * <p> Presumes this Command runs while a button is help
   * and ends when released.
   * @param shooter
   * @param distance in meters
   */
  public AdvancedShoot(Shooter shooter, Drivetrain drive) {
    this.shooter = shooter;
    this.drive = drive;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterSpeed(shooter.getRPM()+2*velocityIncrease);
    shooting = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    velocity = shooter.getRPM();
  if (!shooting){
    shooter.setShooterSpeed(velocity+2*velocityIncrease);
    if(shooter.getShooterVelocity()> velocity+2*velocityIncrease-shootSpeedDeadband){
    timer.start();
    shooter.setConveyorSpeed(ShooterConstants.conveyorShootSpeed);
    shooting = true;
      }
    }
    if (timer.hasElapsed(.75))shooter.setShooterSpeed(velocity);
    /*if (!shooting){
      if(shooter.getShooterVelocity()> velocity-shootSpeedDeadband){
        shooter.setConveyorSpeed(ShooterConstants.conveyorShootSpeed);
        shooting = true;
      }
    }*/
    /*if (shooting && shooter.getShooterVelocity() < velocity-shootSpeedDeadband){
      shotCount++;
      if (shotCount>5) shooter.setShooterSpeed(velocity+velocityIncrease);
    }*/
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
