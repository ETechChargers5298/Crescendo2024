// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.utils.ModuleConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    // Drive & turn motor
  private final CANSparkMax driveMotor;
  private final CANSparkMax turnMotor;

  // Relative & Absolute Encoders
  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turnEncoder;

  // PID Controllers for drive & turn motors
  private final SparkPIDController drivePIDController;
  private final SparkPIDController turnPIDController;

  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  public final ModuleConfig config;

  /**
   * Constructs a SwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public SwerveModule(ModuleConfig config) {
    
    //store the ModuleConfig object for each module
    this.config = config;

    // initialize drive & turn motors
    driveMotor = new CANSparkMax(config.DRIVE_PORT, MotorType.kBrushless);
    turnMotor= new CANSparkMax(config.TURN_PORT, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    driveMotor.restoreFactoryDefaults();
    turnMotor.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
    drivePIDController = driveMotor.getPIDController();
    turnPIDController = turnMotor.getPIDController();
    drivePIDController.setFeedbackDevice(driveEncoder);
    turnPIDController.setFeedbackDevice(turnEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    driveEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve APIs.
    turnEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    turnEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    turnPIDController.setPositionPIDWrappingEnabled(true);
    turnPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    turnPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    drivePIDController.setP(ModuleConstants.kDrivingP);
    drivePIDController.setI(ModuleConstants.kDrivingI);
    drivePIDController.setD(ModuleConstants.kDrivingD);
    drivePIDController.setFF(ModuleConstants.kDrivingFF);
    drivePIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    turnPIDController.setP(ModuleConstants.kTurningP);
    turnPIDController.setI(ModuleConstants.kTurningI);
    turnPIDController.setD(ModuleConstants.kTurningD);
    turnPIDController.setFF(ModuleConstants.kTurningFF);
    turnPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

    // Set the motors to coast (instead of brake)
    driveMotor.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    turnMotor.setIdleMode(ModuleConstants.kTurningMotorIdleMode);

    // Set the limit for the max amount of current each motor can pull
    driveMotor.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    turnMotor.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    // Burn the new Java data into the Sparkmaxes 
    //If a SPARK MAX browns out during operation, it will maintain the above configurations.
    driveMotor.burnFlash();
    turnMotor.burnFlash();

    desiredState.angle = new Rotation2d(turnEncoder.getPosition());
    driveEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(driveEncoder.getVelocity(),
        new Rotation2d(turnEncoder.getPosition() - config.ANGULAR_OFFSET));
  }


  /**
   * Gets the position of the swerve module
   * @author Aiden Sing
   * @return the position
   */ 
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      driveEncoder.getPosition(), 
      new Rotation2d(turnEncoder.getPosition() - config.ANGULAR_OFFSET));
  }

  public double getTurnRadians(){
  return getPosition().angle.getRadians();
  }




  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(config.ANGULAR_OFFSET));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(turnEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivePIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turnPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    this.desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }

  public void updateTelemetry() {
    SmartDashboard.putNumber(config.NAME + " Angle Degrees", getPosition().angle.getDegrees());
    SmartDashboard.putNumber(config.NAME + " Angle Radians", getTurnRadians());
    SmartDashboard.putNumber(config.NAME + " Drive Position", getPosition().distanceMeters);
  }



}
