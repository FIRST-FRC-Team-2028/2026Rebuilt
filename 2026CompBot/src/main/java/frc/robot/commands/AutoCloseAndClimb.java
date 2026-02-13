package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PixyCamReader;

public class AutoCloseAndClimb extends Command {
    /** Presuming the robot is close to and facing away from the tower,
     * close any remaining distance and climb to level one
     */
    public AutoCloseAndClimb(PixyCamReader pixy, Drivetrain drive, Climber climber) {
        Commands.parallel(Commands.race(new DriveTowardTower(pixy, drive), // Drive into tower, ensuring alignment
                                        Commands.waitSeconds(4.)),               // quit when robot off the ground
                        Commands.sequence(Commands.waitSeconds(1.),              // wait til robot has closed the difference 
                        new MoveClimber(climber, Constants.ClimberConstants.climbLevel1))); // climb to level one
    }
    /** Presuming the robot is close to and facing away from the tower,
     * close any remaining distance and climb to desired level
     */
    public AutoCloseAndClimb(PixyCamReader pixy, Drivetrain drive, Climber climber, double level) {
        Commands.parallel(Commands.race(new DriveTowardTower(pixy, drive), // Drive into tower, ensuring alignment
                                        Commands.waitSeconds(4.)),               // quit when robot off the ground
                        Commands.sequence(Commands.waitSeconds(1.),              // wait til robot has closed the difference 
                        new MoveClimber(climber, level))); // climb to desired level
    }
}