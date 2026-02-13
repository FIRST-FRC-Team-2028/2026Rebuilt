// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PixyCamReader extends SubsystemBase {

  AnalogInput analog, analogExist;
  DigitalInput tExist;
  double position;
  boolean targetVisible;

  /** Creates a new ExampleSubsystem. */
  public PixyCamReader() {
    analog = new AnalogInput(3);
    //analogExist = new AnalogInput(1);
    tExist = new DigitalInput(0);
    
  }



  /**Reads voltage [Far Left = 0.0V - Far Right = 3.3V] from analog. (Already translated to be between -1 & 1). */
  public double getPosition() {
    position = (0.6 * analog.getVoltage()) -1.; 
    return position;
  }

  public boolean isVisible() {
    return tExist.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("SeeTarget", isVisible());
    SmartDashboard.putNumber("TargetPosition",getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
