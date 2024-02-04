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
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.utils.ModuleConfig;


public class SwerveModule extends SubsystemBase {

  // Drive & turn motor
  private final CANSparkMax driveMotor;
  private final CANSparkMax turnMotor;

  // Relative & Absolute Encoders
  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turnEncoder;

  public final SparkPIDController driveController;
  public final SparkPIDController turnController;

  public SwerveModuleState desiredState;
  public double angularOffset;
  
  public final ModuleConfig config;

  /** Creates a new SwerveModule. */
  public SwerveModule(ModuleConfig config) {

    // initialize drive & turn motors
    driveMotor = new CANSparkMax(config.DRIVE_PORT, MotorType.kBrushless);
    turnMotor= new CANSparkMax(config.TURN_PORT, MotorType.kBrushless);

    //clear off the SparkMaxes to default values
    driveMotor.restoreFactoryDefaults();
    turnMotor.restoreFactoryDefaults();

    // get drive & turn encoders
    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

    /**** TEST**/
    System.out.println("Initialized! " + turnEncoder.getPosition());

    // setup PID controllers for drive & turn with appropriate sensors
    driveController = driveMotor.getPIDController();
    driveController.setFeedbackDevice(driveEncoder);
    turnController = SwerveConstants.TURN_PID.getConfiguredController(turnMotor, turnEncoder);
    
    // converting the drive factors to meters and the turn factors to radians
    driveEncoder.setVelocityConversionFactor(((Math.PI * SwerveConstants.WHEEL_DIAMETER) / SwerveConstants.GEER_RATTIOLI) / 60); 
    driveEncoder.setPositionConversionFactor((Math.PI * SwerveConstants.WHEEL_DIAMETER) / SwerveConstants.GEER_RATTIOLI);
    turnEncoder.setVelocityConversionFactor((Math.PI * 2) / 60);
    turnEncoder.setPositionConversionFactor(Math.PI * 2);

    // Set the motors to coast (instead of brake)
    driveMotor.setIdleMode(IdleMode.kCoast);
    turnMotor.setIdleMode(IdleMode.kCoast);

    // Set the limit for the max amount of current each motor can pull
    driveMotor.setSmartCurrentLimit(50);
    turnMotor.setSmartCurrentLimit(20);

    // Set the zero offset with code with saved value from the ModuleConfig
    turnEncoder.setZeroOffset(config.OFFSET);

    //???  Goes crazy without this
    turnEncoder.setInverted(Constants.SwerveConstants.TURN_INVERSION);

    // Burn the new Java data into the Sparkmaxes
    driveMotor.burnFlash();
    turnMotor.burnFlash();

    // Set starting angle of wheels
    desiredState = new SwerveModuleState();
    desiredState.angle = new Rotation2d(turnEncoder.getPosition());
    
    // 
    //this.angularOffset = angularOffset;
    this.config = config;

    driveEncoder.setPosition(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turnEncoder.getPosition() + angularOffset));
}

/**
 * Gets the position of the swerve module
 * @author Aiden Sing
 * @return the position
 */ 
public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(turnEncoder.getPosition() + angularOffset));
}

public double getTurnRadians(){
  return getPosition().angle.getRadians();
}


int count =0;
double startVal = 0;
double endVal = 0;
/**
 * Moves the swerve module
 * @author Aiden Sing
 * @param desiredState Where the module should go
 */
public void setState(SwerveModuleState desiredState) {

  //printout of angle before setState() is run
  if(config.NAME.equals("BL") && count ==0){
  startVal = getTurnRadians();
  }

  //SwerveModuleState correctedDesiredState = new SwerveModuleState();
    //correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    // correctedDesiredState.angle = desiredState.angle.minus(Rotation2d.fromRadians(angularOffset));

    // optimizing the state of the angle
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, new Rotation2d(getTurnRadians()));
    // running the optimized state
    driveMotor.set(optimizedState.speedMetersPerSecond / SwerveConstants.TOP_SPEED);

    driveController.setReference(optimizedState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turnController.setReference(optimizedState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
    //if(Math.abs(desiredState.angle.minus(this.desiredState.angle).getRadians()) > SwerveConstants.ANGLE_THRESHOLD) {
        
    //}

    this.desiredState = desiredState;

    // printout first value after setState() is run
    if(count ==0){
    endVal = getTurnRadians();
    }
    count++;
    System.out.println(startVal + " ---> "+ endVal);
  }

public void updateTelemetry() {
    SmartDashboard.putNumber(config.NAME + " Angle Degrees", getPosition().angle.getDegrees());
    SmartDashboard.putNumber(config.NAME + " Angle Radians", getTurnRadians());
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
