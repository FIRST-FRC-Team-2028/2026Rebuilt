// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final SparkMax centerShooter, leftShooter, /*rightShooter,*/ conveyor;
  private final SparkMaxConfig  center_Config, left_Config, /*right_Config,*/ conveyor_Config;
  private final RelativeEncoder center_Encoder, conveyor_Encoder;
  private final SparkClosedLoopController center_ClosedLoopController, conveyor_ClosedLoopController;
  double setShootSpeed;
  /** Manupulates scoring element: fuel
   * <p>Methods:<ul>
   * <li>{@code setShooterSpeed} - Sets the speed for the shooter using PID controller on velocity control type
   * <li>{@code getShooterVelocity} - Returns the RPM of the shooter wheels
   * <li>{@code setConveyorSpeed} - Sets the speed for the conveyor using PID controller on velocity
   * <li>{@code setFeedSpeed} - Sets the speed for the feed using PID controller on velocity
   * <li>{@code stopShooting} - Stops all of the motors involved with shooting
   * <li>{@code getShooterRPM} - Computes the desired shooter wheel speed in RPM based on horizontal distance
   * to the target.
   * <li>{@code calculateExitVelocity} - Calculates the linear exit velocity required for a fuel to reach
   * the target from a given horizontal distance.
   * <li>{@code velocityToRPM} - Converts a linear exit velocity into shooter wheel RPM. Using <pre>
   * v = ωr
   * </pre>
   * </ul>
   * </p>
   */
  public Shooter() {
    leftShooter = new SparkMax(CANIDS.leftShooter, MotorType.kBrushless);
    centerShooter = new SparkMax(CANIDS.centerShooter, MotorType.kBrushless);
    //rightShooter = new SparkMax(CANIDS.rightShooter, MotorType.kBrushless);
    conveyor = new SparkMax(CANIDS.conveyor, MotorType.kBrushless);

    left_Config = new SparkMaxConfig();
    center_Config = new SparkMaxConfig();
    //right_Config = new SparkMaxConfig();
    conveyor_Config = new SparkMaxConfig();

    center_Encoder = centerShooter.getEncoder();
    conveyor_Encoder = conveyor.getEncoder();

    center_ClosedLoopController = centerShooter.getClosedLoopController();
    conveyor_ClosedLoopController = conveyor.getClosedLoopController();


    center_Config
      .idleMode(IdleMode.kCoast);
    center_Config.encoder
      .velocityConversionFactor(ShooterConstants.shooterVelocityConversionFactor);
    center_Config.closedLoop
      .pid(ShooterConstants.shooterP, ShooterConstants.shooterI, ShooterConstants.shooterD);
    
    left_Config.idleMode(IdleMode.kCoast).follow(CANIDS.centerShooter);
    //right_Config.follow(CANIDS.centerShooter);

    conveyor_Config
      .idleMode(IdleMode.kCoast);
    conveyor_Config.encoder
      .velocityConversionFactor(ShooterConstants.conveyorVelocityConversionFactor);
    conveyor_Config.closedLoop
      .pid(ShooterConstants.conveyorP, ShooterConstants.conveyorI, ShooterConstants.conveyorD);


    leftShooter.configure(left_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    centerShooter.configure(center_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //rightShooter.configure(right_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    conveyor.configure(conveyor_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

 @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Speed (RPM)", center_Encoder.getVelocity());
  }
  /** Set the speed for the shooter using VBUS - test control */
  public void setShooterVbus(double Speed){
    centerShooter.set(Speed);
  }

  //public void goConvey(double speed){conveyor.set(speed);}
  //public double getConveySpeed(){return conveyor_Encoder.getVelocity();}

  /**Sets the speed for the shooter using PID controller on velocity control type
   * @param Speed in RPM
   */
  public void setShooterSpeed(double Speed){
    center_ClosedLoopController.setSetpoint(Speed, ControlType.kVelocity);
    setShootSpeed = Speed;
  }

  public void incrementShootSpeed(double increment){
    setShooterSpeed(setShootSpeed+increment);
  }  
  /** Returns the RPM of the shooter wheels */
  public double getShooterVelocity(){
    return center_Encoder.getVelocity();
  }

  /** Sets the speed for the conveyor using PID controller on velocity
   * @param Speed in RPM
   */
  public void setConveyorSpeed(double Speed){
    conveyor.set(Speed);
    //conveyor_ClosedLoopController.setSetpoint(Speed, ControlType.kVelocity);
  }
  

  /** Stops all of the motors involved with shooting */
  public void stopShooting(){
    centerShooter.stopMotor();
    conveyor.stopMotor();
  }


  //Shooter RPM Math
    /**
 * Computes the desired shooter wheel speed in RPM based on horizontal distance
 * to the target.
 *
 * <p>This method:
 * <ul>
 *   <li>Clamps the input distance to a physically valid range</li>
 *   <li>Computes the required exit velocity using projectile motion</li>
 *   <li>Applies empirical correction for drag and compression</li>
 *   <li>Converts linear exit velocity into wheel RPM</li>
 * </ul>
 *
 * <p>Distance is assumed to be measured horizontally from the shooter to the
 * target plane (e.g., via vision).
 *
 * @param distanceMeters Horizontal distance to the target in meters
 * @return Shooter wheel speed in RPM, or {@code Double.NaN} if wheel diameter
 *         has not yet been configured
 */
    public double getShooterRPM(double distanceMeters) {

        // Clamp distance to something physically possible
        double minDistance = ShooterConstants.delta_H / Math.tan(Math.toRadians(ShooterConstants.shooter_angle_deg)) + 0.05;
        double clampedDistance = Math.max(distanceMeters, minDistance);

        if (ShooterConstants.max_shooting_distance_meters > 0.0) {
            clampedDistance = Math.min(clampedDistance, ShooterConstants.max_shooting_distance_meters);
        }

        // Calculate exit velocity (m/s)
        double velocity = calculateExitVelocity(clampedDistance);
        SmartDashboard.putNumber("ClampedDistance", clampedDistance);
        SmartDashboard.putNumber("RPM", velocityToRPM(clampedDistance));

        // Convert velocity to RPM
        return velocityToRPM(velocity);
    }

    /**
 * Calculates the linear exit velocity required for a fuel to reach
 * the target from a given horizontal distance.
 *
 * <p>This calculation assumes:
 * <ul>
 *   <li>A fixed shooter angle</li>
 *   <li>Known height difference between shooter and target</li>
 *   <li>Ideal projectile motion with an empirical correction factor applied</li>
 * </ul>
 *
 * <p>The returned velocity includes a tunable "fudge factor" to account for
 * aerodynamic drag, wheel slip, and ball compression losses.
 *
 * @param distanceMeters Horizontal distance to the target in meters
 * @return Required exit velocity in meters per second
 */
  private double calculateExitVelocity(double distanceMeters) {

    double thetaRad = Math.toRadians(ShooterConstants.shooter_angle_deg);

    double numerator = ShooterConstants.gravity * distanceMeters * distanceMeters;
    double denominator = 2.0 * Math.pow(Math.cos(thetaRad), 2)
                * (distanceMeters * Math.tan(thetaRad) - ShooterConstants.delta_H);

    double idealVelocity = Math.sqrt(numerator / denominator);
    
    // Apply real-world correction
    return idealVelocity * ShooterConstants.velocity_fudge_factor;
  }
  /**
 * Converts a linear exit velocity into shooter wheel RPM.
 *
 * <p>Uses the relationship:
 * <pre>
 * v = ωr
 * </pre>
 * where {@code v} is linear velocity and {@code r} is wheel radius.
 *
 * <p>If the shooter wheel diameter has not yet been configured, this method
 * returns {@code Double.NaN} to clearly indicate a configuration error.
 *
 * @param velocityMetersPerSecond Linear exit velocity in meters per second
 * @return Shooter wheel speed in RPM, or {@code Double.NaN} if wheel diameter
 *         is undefined
 */
  private double velocityToRPM(double velocityMetersPerSecond) {
    double wheelRadius = ShooterConstants.wheel_diameter_meter / 2.0;

    if (wheelRadius <= 0.0) {
        // Wheel not defined yet — return NaN so error is obvious
         return Double.NaN;
    }
    // v = ωr → RPM conversion
     return (velocityMetersPerSecond * 60.0) / (2.0 * Math.PI * wheelRadius);
    }
}
