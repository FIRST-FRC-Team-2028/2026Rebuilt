package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PixyCamReader;

public class DriveTowardTower extends Command{
  private final PixyCamReader pixy;
  private final Drivetrain drive;
  //private int towerColor;
  double tPosition; //Tower Position
  Pose2d rPosition; //Robot Position
  double rPositionX; //The (X) from the robot Pose2D.
  final double headingkp = (DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond*0.1)*.1 ; //The control range divided our sensor error of 10 degrees. With a knock down factor of .1
  final double ykp = (DriveConstants.kTeleDriveMaxSpeedMetersPerSecond/1.)*.1; //The control range(Drive Max Speed) over the Pixy range[-1,1]. With a knock down factor of .1
  final double pidToleranceH = 1.; //Degrees
  final double pidToleranceY = .2; // Out of a Pixy [-1,1]
  PIDController rPidControllerHeading, rPidControllerY, rPidControllerX;

  double xSpeed, ySpeed, turningSpeed;
  
  /* Drive towards tower. 
   * Use camera position to align vertical post.
   * Maintain desired heading.
   * 
   * End when robot is aligned with tower? or keep pushing toward tower in case climber tries to push away? TODO
   */

  /**Use PixyCamera trained for tower to get close enough to climb */
  public DriveTowardTower(PixyCamReader pixy, Drivetrain drive/* , int towerColor*/) {
        this.drive = drive;
        this.pixy = pixy;
        //this.towerColor =towerColor;
        rPidControllerHeading = new PIDController(headingkp, 0., 0.);
        rPidControllerY = new PIDController(ykp, 0, 0);
        addRequirements(drive,pixy);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   drive.BreakMode();  //MrG asks why? TODO
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     tPosition = -pixy.getPosition(); // pixy right is robot left
     //rPosition = drive.getPoseEstimatorPose();
     //rPositionX = rPosition.getX();
     ySpeed = rPidControllerY.calculate(tPosition, 0.);
     turningSpeed = rPidControllerHeading.calculate(drive.getHeading().getDegrees(), 0.);
     xSpeed = -0.02; // robot is facing away from tower; drive backwards
     ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
     drive.drive(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // MrG asks, initialize set drive.brakemode. Do you want that to persist after this command ends? TODO
  }

  // Continue until interrupted.
  @Override
  public boolean isFinished() {
    //if (Math.abs(rPidControllerY.getError()) < pidToleranceY && Math.abs(rPidControllerHeading.getError()) < pidToleranceH) return true;
    return false;
  }
}
