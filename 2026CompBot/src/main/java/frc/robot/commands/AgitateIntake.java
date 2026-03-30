// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AgitateIntake extends Command {
  private final Intake intake;
  double deadband = 5; //Degrees
  double setPoint = IntakeConstants.JointPickupPosition;
  double setpointF = IntakeConstants.JointFPickupPosition;
  Timer timer;
  /** Shake hopper
   * <p> Presumes this Command is controlled by holding a button,
   * and ends when button is released.
   * @param intake
   */
  public AgitateIntake(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setJointPosition(setPoint, setpointF);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.getJointPosition()<setPoint+deadband && intake.getJointPosition()>setPoint-deadband){
      if (setPoint == IntakeConstants.JointPickupPosition){
        setPoint = IntakeConstants.JointAgitatePosition;
        setpointF = IntakeConstants.JointFAgitatePosition;
        if(timer.hasElapsed(4)){
          setPoint=setPoint+30;
          setpointF=setpointF+30;
        }
      } else {
        setPoint = IntakeConstants.JointPickupPosition;
        setpointF = IntakeConstants.JointFPickupPosition;
      }
      intake.setJointPosition(setPoint, setpointF); 
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setJointPosition(IntakeConstants.JointPickupPosition, IntakeConstants.JointFPickupPosition);
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
