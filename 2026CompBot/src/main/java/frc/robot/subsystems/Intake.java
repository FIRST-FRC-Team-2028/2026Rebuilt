// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final SparkMax rollers;
  private final SparkMax joint;
  private final SparkMaxConfig rollers_Config;
  private final SparkMaxConfig joint_Config;
  private final RelativeEncoder rollers_Encoder;
  private final RelativeEncoder joint_Encoder;
  private final SparkClosedLoopController joint_Controller;
  /** Creates a new Intake. */
  public Intake() {
    rollers = new SparkMax(Constants.CANIDS.roller, MotorType.kBrushless);
    joint = new SparkMax(Constants.CANIDS.joint, MotorType.kBrushless);
    
    rollers_Config = new SparkMaxConfig();
    joint_Config = new SparkMaxConfig();

    rollers_Config
      .idleMode(IdleMode.kCoast);
    rollers_Config.encoder
      .velocityConversionFactor(IntakeConstants.RollerVelocityConversionFactor);

    joint_Config
      .idleMode(IdleMode.kCoast);
    joint_Config.encoder
      .positionConversionFactor(IntakeConstants.JointPositionConversionFactor);
    joint_Config.closedLoop
      .pid(IntakeConstants.jointP, IntakeConstants.jointI, IntakeConstants.jointD);
    joint_Config.softLimit
      .forwardSoftLimit(IntakeConstants.jointForwardSoftLimit)
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimit(IntakeConstants.jointReverseSoftLimit)
      .reverseSoftLimitEnabled(true);

    rollers.configure(rollers_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    joint.configure(joint_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rollers_Encoder = rollers.getEncoder();
    joint_Encoder = joint.getEncoder();

    joint_Controller = joint.getClosedLoopController();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /**Set the rollers to a speed between -1 and 1 */
  public void rollers(double speed){
    rollers.set(speed);
  }
  /**Sets the position of the Joint in degrees */
  public void PositionJoint(double position){
    position = Units.degreesToRotations(position);
    joint_Controller.setSetpoint(position, ControlType.kPosition);
  }  
  /**Returns the joint position in degrees */
  public double getJointPosition() {
    return Units.rotationsToDegrees(joint_Encoder.getPosition());
  }

  public Command runIntake(){
    return new InstantCommand(()->rollers(0.5));
  }
   public Command runIntake(double speed){
    return new InstantCommand(()->rollers(speed));
  }
  public Command stopIntake(){
    return new InstantCommand(()->rollers.stopMotor());
  }

}
