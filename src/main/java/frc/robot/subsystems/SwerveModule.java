// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;
import frc.robot.utils.ModuleConfig;


public class SwerveModule extends SubsystemBase {

  // Drive & turn motor
  private final CANSparkMax driveMotor;
  private final CANSparkMax turnMotor;

  // Relative & Absolute Encoders
  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turnEncoder;

  public final SparkPIDController turnController;

  public SwerveModuleState currentState;

  public final ModuleConfig config;

  /** Creates a new SwerveModule. */
  public SwerveModule(ModuleConfig config) {

    // initialize drive & turn motors
    driveMotor = new CANSparkMax(config.DRIVE_PORT, MotorType.kBrushless);
    turnMotor= new CANSparkMax(config.TURN_PORT, MotorType.kBrushless);

    // get drive & turn encoders
    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

    // converting the drive factors to meters and the turn factors to radians

     driveEncoder.setVelocityConversionFactor(((Math.PI * SwerveConstants.WHEEL_DIAMETER) / SwerveConstants.GEER_RATTIOLI) / 60); 
     driveEncoder.setPositionConversionFactor((Math.PI * SwerveConstants.WHEEL_DIAMETER) / SwerveConstants.GEER_RATTIOLI);

    turnEncoder.setVelocityConversionFactor((Math.PI * 2) / 60);
    turnEncoder.setPositionConversionFactor(Math.PI * 2);

    turnController = SwerveConstants.TURN_PID.getConfiguredController(turnMotor, turnEncoder);
    
    this.config = config;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turnEncoder.getPosition()));
}

/**
 * Gets the position of the swerve module
 * @author Aiden Sing
 * @return the position
 */ 
public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(turnEncoder.getPosition()));
}

/**
 * Moves the swerve module
 * @author Aiden Sing
 * @param desiredState Where the module should go
 */
public void setState(SwerveModuleState desiredState) {
    // updating the desired state using the angle offset
    //desiredState.angle.plus(Rotation2d.fromRadians(angleOffset));

    // optimizing the state of the angle
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getState().angle);

    // running the optimized state
    driveMotor.set(optimizedState.speedMetersPerSecond / SwerveConstants.TOP_SPEED);

    if(Math.abs(desiredState.angle.minus(currentState.angle).getRadians()) > SwerveConstants.ANGLE_THRESHOLD) {
        turnController.setReference(optimizedState.angle.getRadians(), ControlType.kPosition);
    }

    currentState = getState();
}

public void updateTelemetry() {
    SmartDashboard.putNumber(config.NAME + " Angle Degrees", getPosition().angle.getDegrees());
    SmartDashboard.putNumber(config.NAME + " Angle Radians", getPosition().angle.getRadians());

    SmartDashboard.putNumber(config.NAME + " Drive Position", getPosition().distanceMeters);
}

public void resetEncoder() {
  driveEncoder.setPosition(0.0);
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
