// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
//import com.revrobotics.spark.config.ClosedLoopConfig;
//import com.revrobotics.spark.config.EncoderConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final SparkFlex rollers;
  private final SparkMax jointLead, jointFollow;
  private final SparkFlexConfig rollers_Config;
  private final SparkMaxConfig joint_Config, jointF_Config;
  private final RelativeEncoder rollers_Encoder;
  private final RelativeEncoder joint_Encoder, joint_Encoder2;
  private final SparkClosedLoopController joint_Controller;
  /** Picks up the scoring element: fuel
   * <p>Methods: <ul>
   * <li>{@code rollers} - Sets the rollers to a speed
   * <li>{@code setJointPosition} - Sets the joint position in degrees
   * <li>{@code getJointPosition} - Gets the joint position in degrees
   * <li>{@code runIntake} - Runs the rollers at half speed
   * <li>{@code runIntake(double speed)} - Runs the rollers at inputed speed
   * <li>{@code stopIntake} - Stops the rollers 
   */
  public Intake() {
    rollers = new SparkFlex(Constants.CANIDS.roller, MotorType.kBrushless);
    jointLead = new SparkMax(Constants.CANIDS.jointLead, MotorType.kBrushless);
    jointFollow = new SparkMax(Constants.CANIDS.jointFollow, MotorType.kBrushless);
    
    rollers_Config = new SparkFlexConfig();
    joint_Config = new SparkMaxConfig();
    jointF_Config = new SparkMaxConfig();


    rollers_Config
      .idleMode(IdleMode.kCoast);
    rollers_Config.encoder
      .velocityConversionFactor(IntakeConstants.RollerVelocityConversionFactor);

    joint_Config
      .idleMode(IdleMode.kCoast);
    joint_Config.encoder
      .positionConversionFactor(IntakeConstants.JointPositionConversionFactor);  // cannot change sign; out direction is encoder negative
    jointF_Config.encoder
      .positionConversionFactor(IntakeConstants.JointPositionConversionFactor);
    joint_Config.closedLoop
      .pid(IntakeConstants.jointP, IntakeConstants.jointI, IntakeConstants.jointD);
    joint_Config.softLimit
      .forwardSoftLimit(IntakeConstants.jointForwardSoftLimit)  // forward is retracted
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimit(IntakeConstants.jointReverseSoftLimit)  // reverse is deployed
      .reverseSoftLimitEnabled(true);
    jointF_Config.follow(jointLead,true);

    rollers.configure(rollers_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    jointLead.configure(joint_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    jointFollow.configure(jointF_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rollers_Encoder = rollers.getEncoder();
    joint_Encoder = jointLead.getEncoder();
    joint_Encoder2 = jointFollow.getEncoder();

    joint_Controller = jointLead.getClosedLoopController();
    joint_Encoder.setPosition(IntakeConstants.jointForwardSoftLimit);

  }

  @Override
  public void periodic() {}
    // This method will be called once per scheduler run
  
  /** Set the rollers to a speed between -1 and 1 */
  public void rollers(double speed){
    rollers.set(speed);
  }
  public double getRollerSpeed(){return rollers_Encoder.getVelocity();}

  /**make joint motor move Vbus - for testing */
  public void goJoint(double speed){
    jointLead.set(speed);
  }
  public void goJointF(double speed){
    jointFollow.set(speed);
  }
  /** Sets the position of the Joint in degrees
   * where positive is retracted, negative is deployed
   */
  public void setJointPosition(double position){
    joint_Controller.setSetpoint(position, ControlType.kPosition);
  }  
  /**NO Returns the joint position in degrees */
  public double getJointPosition() {
    return joint_Encoder.getPosition();
  }
  public double getJointPosition2() {
    return joint_Encoder2.getPosition();
  }
  public void switchSoftLimits(boolean offon, boolean resetZero){
    joint_Config.softLimit.forwardSoftLimitEnabled(offon);
    joint_Config.softLimit.reverseSoftLimitEnabled(offon);
    jointLead.configure(joint_Config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    if (resetZero) joint_Encoder.setPosition(IntakeConstants.JointUpPosition);
  }
  public void setJointIdleMode(IdleMode mode){
    joint_Config.idleMode(mode);
    jointLead.configure(joint_Config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

  }
  /**setPID values */
  public void setJointPID(double p,double i, double d){
    joint_Config.closedLoop
      .pid(p,i,d);
    jointLead.configure(joint_Config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /** Runs the rollers at {@code .5} speed */
  public Command runIntake(){
    return new InstantCommand(()->rollers(0.5));
  }

  /** Runs the intake wheels at a inputed speed
   * @param speed to set the rollers between -1 and 1
   */
   public Command runIntake(double speed){
    return new InstantCommand(()->rollers(speed));
  }

  /** Stops the rollers
   */
  public Command stopIntake(){
    return new InstantCommand(()->rollers.stopMotor());
  }

  public void resetJointEncoder(){
    joint_Encoder.setPosition(IntakeConstants.jointForwardSoftLimit);
    //joint_Encoder2.setPosition(IntakeConstants.jointReverseSoftLimit);
  }

}
