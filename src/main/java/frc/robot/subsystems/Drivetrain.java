// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class Drivetrain extends SubsystemBase {

  // an instance variable for the drivetrain
  private static Drivetrain instance;

  private final SwerveModule[] modules;

  private final SwerveDriveKinematics driveKinematics;
  private final SwerveDriveOdometry driveOdometry;
  
  private AHRS navX;

  //private Pose2d pose;
  private Field2d field;

  private boolean fieldCentric;

  /** Creates a new Drivetrain. */
  private Drivetrain() {
    modules = new SwerveModule[4];

    modules[0] = new SwerveModule(SwerveConstants.SWERVE_FL, 90);
    modules[1] = new SwerveModule(SwerveConstants.SWERVE_FR, 0);
    modules[2] = new SwerveModule(SwerveConstants.SWERVE_BR, 90);
    modules[3] = new SwerveModule(SwerveConstants.SWERVE_BL, 0);

    driveKinematics = new SwerveDriveKinematics(
      new Translation2d(SwerveConstants.WHEEL_BASE / 2, SwerveConstants.TRACK_WIDTH / 2),
      new Translation2d(SwerveConstants.WHEEL_BASE / 2, -SwerveConstants.TRACK_WIDTH / 2),
      new Translation2d(-SwerveConstants.WHEEL_BASE / 2, -SwerveConstants.TRACK_WIDTH / 2),
      new Translation2d(-SwerveConstants.WHEEL_BASE / 2, SwerveConstants.TRACK_WIDTH / 2));

      navX = new AHRS(SPI.Port.kMXP);
      driveOdometry = new SwerveDriveOdometry(driveKinematics, getHeading(), swerveModulepos(), new Pose2d(0, 0, new Rotation2d()));

      //pose = driveOdometry.getPoseMeters();
      field = new Field2d();

      navX.reset();

      fieldCentric = true;
  }

  /**
   * Gets the single instance of drivetrain
   * @author Aiden Sing
   * @return the instance of the drivetrain
   */
  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
    }
    return instance;
  }

  /**
   * Making a drive function to make the speed for drive a fraction of total
   * @author Aiden Sing
   * @param xSpeed speed of the robot front to back
   * @param ySpeed speed of robot left to right
   * @param rotSpeed speed of robot turning
   */
  public void drive(double xSpeed, double ySpeed, double rotSpeed) {
    xSpeed *= SwerveConstants.TOP_SPEED;
    ySpeed *= SwerveConstants.TOP_SPEED;
    rotSpeed *= SwerveConstants.TOP_ANGULAR_SPEED;

    ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
    
    SwerveModuleState[] states = driveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.TOP_SPEED);

    // setting the state for each module as an array
    for(int i = 0; i < modules.length; i++) {
      modules[i].setState(states[i]);
    }
  }

  public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldCentric) {
    xSpeed *= SwerveConstants.TOP_SPEED;
    ySpeed *= SwerveConstants.TOP_SPEED;
    rotSpeed *= SwerveConstants.TOP_ANGULAR_SPEED;

    ChassisSpeeds speeds = fieldCentric ? 
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, getHeading()) : 
      new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
    
    SwerveModuleState[] states = driveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.TOP_SPEED);

    for(int i = 0; i < modules.length; i++) {
      modules[i].setState(states[i]);
    }
  }

  public void resetIMU() {
    navX.reset();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(navX.getAngle());
  }

  public float getPitch() {
    return navX.getPitch();
  }

  public float getRoll() {
    return navX.getRoll();
  }

  // public Rotation2d getHeading() {
  //   return Rotation2d.fromRadians(MathUtil.angleModulus(getRawHeading().getRadians()));
  // }

  public boolean getFieldCentric() {
    return fieldCentric;
  }

  public void setFieldCentric(boolean fieldCentric) {
    this.fieldCentric = fieldCentric;
  }

  public void setModuleState(SwerveModuleState[] states) {

    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.TOP_SPEED);

    for(int i = 0; i < modules.length; i++) {
      modules[i].setState(states[i]);
    }
  }

  public SwerveModulePosition[] swerveModulepos() {
    SwerveModulePosition[] modulePosition = new SwerveModulePosition[4];
    for(int i = 0; i < modules.length; i++) {
      modulePosition[i] = modules[i].getPosition();
    }

    return modulePosition;
  }

  public SwerveDriveOdometry getOdometry() {
    return driveOdometry;
  }

  public SwerveDriveKinematics getKinematics() {
    return driveKinematics;
  }

  public void resetOdometry(Pose2d newPose) {
    driveOdometry.resetPosition(getHeading(), swerveModulepos(), newPose);
  }

  public void updateOdometry() {
    driveOdometry.update(getHeading(), swerveModulepos());
  }

  public Pose2d getPose2d() {
    return driveOdometry.getPoseMeters();
  }

  public Field2d getField() {
    return field;
  }

  // Assuming this method is part of a drivetrain subsystem that provides the necessary methods


  public void updateTelemetry() {
    for(int i = 0; i < modules.length; i++) {
      modules[i].updateTelemetry();
    }

    // SmartDashboard.putNumber("Robot Angle", getHeading().getDegrees());

    SmartDashboard.putNumber("xOdometry", getPose2d().getX());
    SmartDashboard.putNumber("yOdometry", getPose2d().getY());
    SmartDashboard.putNumber("rotOdometry", getPose2d().getRotation().getDegrees());
    // SmartDashboard.putNumber("pitch", getPitch());
    // SmartDashboard.putNumber("roll", getRoll());

    //SmartDashboard.putData("Odometry Field", field);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateTelemetry();
    updateOdometry();
  }
}
