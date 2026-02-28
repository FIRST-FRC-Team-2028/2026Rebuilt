// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  private final Shooter shooter;
  double shootSpeedDeadband = 100;
  boolean shooting = false;
  double shotCount = 0;
  double velocityIncrease = 200, speed;
  /** Runs procedure to Shoot
   * <p> Presumes non-negligible time for shooter wheels to get to speed.
   * <p> Presumes this Command is controlled by holding a button, 
   * and ends when button is released.
   */
  public Shoot(Shooter shooter, double speed) {
    this.shooter = shooter;
    this.speed = speed;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterSpeed(speed+(4.5*velocityIncrease));
    shotCount = 0;
    shooting = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!shooting){
      if(shooter.getShooterVelocity()> speed+(4.5*velocityIncrease)-shootSpeedDeadband){
        shooter.setConveyorSpeed(ShooterConstants.conveyorShootSpeed);
        shooter.setShooterSpeed(speed-1*velocityIncrease);
      shooting = true;
      }
    }
    /*if (shooting && shooter.getShooterVelocity() < ShooterConstants.shooterShootSpeed-shootSpeedDeadband){
      shotCount++;
      if (shotCount>5) shooter.setShooterSpeed(ShooterConstants.shooterShootSpeed+velocityIncrease);
    }*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooting();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
