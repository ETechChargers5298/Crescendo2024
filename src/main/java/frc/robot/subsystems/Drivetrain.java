// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
// import edu.wpi.first.wpilibj.ADIS16470_IMU;
// import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import frc.robot.Ports;
import frc.robot.Constants.SwerveConstants;
import frc.robot.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    //---------------FIELDS --------------//

  // an instance variable for the drivetrain
  private static Drivetrain instance;

  private final SwerveModule[] modules;
  private final SwerveDriveKinematics driveKinematics;
  private final SwerveDriveOdometry driveOdometry;
   //private Pose2d pose;
   private Field2d field;
   private boolean fieldCentric;


  // Create SwerveModules
  private final SwerveModule frontLeft = new SwerveModule(SwerveConstants.SWERVE_FL);
  private final SwerveModule frontRight = new SwerveModule(SwerveConstants.SWERVE_FR);
  private final SwerveModule backLeft = new SwerveModule(SwerveConstants.SWERVE_BL);
  private final SwerveModule backRight = new SwerveModule(SwerveConstants.SWERVE_BR);

  // The gyro sensor
  private AHRS navX;
  // private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(SwerveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(SwerveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  //---------------DRIVETRAIN SUBSYSTEM CONSTRUCTOR --------------//
  public Drivetrain() {

    //assign all 4 SwerveModules to an array of modules
    this.modules = new SwerveModule[4];
    modules[0] = frontLeft;
    modules[1] = frontRight;
    modules[2] = backLeft;
    modules[3] = backRight;

    //assign the NavX to be our sensor for rotation
    this.navX = new AHRS(SPI.Port.kMXP);

    //store the shape of the robot wheels
    this.driveKinematics = SwerveConstants.DRIVE_KINEMATICS;

    // Odometry object for tracking robot pose
    this.driveOdometry = new SwerveDriveOdometry(
      SwerveConstants.DRIVE_KINEMATICS,
      getHeading(),
      getSwerveModulePos()
    );


    //driveOdometry = new SwerveDriveOdometry(driveKinematics, getHeading(), swerveModulePos(), new Pose2d(0, 0, new Rotation2d()));

    //pose = driveOdometry.getPoseMeters();
    field = new Field2d();

    navX.reset();

    fieldCentric = true; //default to fieldcentric

  } //end constructor

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

  public boolean getFieldCentric() {
    return fieldCentric;
  }

  public void setFieldCentric(boolean fieldCentric) {
    this.fieldCentric = fieldCentric;
  }

    
  //---------------DRIVE METHODS --------------//

  /**
   * Making a drive function to make the speed for drive a fraction of total
   * @author Aiden Sing
   * @param xSpeed speed of the robot front to back
   * @param ySpeed speed of robot left to right
   * @param rotSpeed speed of robot turning
   */
  public void drive(double xSpeed, double ySpeed, double rotSpeed) {
    drive(xSpeed, ySpeed, rotSpeed, false, true);
  }



  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldcentric Whether the provided x and y speeds are relative to the field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldcentric, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(SwerveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * SwerveConstants.TOP_SPEED;
    double ySpeedDelivered = ySpeedCommanded * SwerveConstants.TOP_SPEED;
    double rotSpeedDelivered = m_currentRotation * SwerveConstants.TOP_ANGULAR_SPEED;

    //var???
    //SwerveModuleState[] 
    // var swerveModuleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
    //     fieldcentric
    //         ? ChassisSpeeds.fromfieldcentricSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
    //         : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.TOP_SPEED);

    //Store an array of speeds for each wheel
    ChassisSpeeds speeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotSpeedDelivered);
    // ChassisSpeeds speeds = fieldCentric ? 
    //   ChassisSpeeds.fromfieldcentricSpeeds(xSpeed, ySpeed, rotSpeed, getHeading()) : 
    //   new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);


    //Store the states of each module
    SwerveModuleState[] swerveModuleStates = driveKinematics.toSwerveModuleStates(speeds);
    
    //cleans up any weird speeds that may be too high after kinematics equation
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.TOP_SPEED);

    // setting the state for each module as an array
    for(int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(swerveModuleStates[i]);
    }
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }


    
  //---------------SWERVEMODULE HELPER METHODS --------------//

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.TOP_SPEED);
    for(int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(desiredStates[i]);
    }
  }

  // method to return all the positions of the 4 modules
  public SwerveModulePosition[] getSwerveModulePos() {
    
    // return new SwerveModulePosition[] {
    //         frontLeft.getPosition(),
    //         frontRight.getPosition(),
    //         backLeft.getPosition(),
    //         backRight.getPosition()
    // };
    
    SwerveModulePosition[] modulePosition = new SwerveModulePosition[4];
    for(int i = 0; i < modules.length; i++) {
      modulePosition[i] = modules[i].getPosition();
    }
    return modulePosition;
  }

  
  
  //---------------NAVX METHODS --------------//

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(navX.getAngle());
    //return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)).getDegrees();
  }

 public double getHeadingRadians() {
    return getHeading().getRadians();
  }

  // Resets the drive encoders to currently read a position of 0.
  public void resetEncoders() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backRight.resetEncoders();
    backLeft.resetEncoders();
  }

  // Zeroes the heading of the robot
  public void zeroHeading() {
    navX.reset();
  }
  public void resetIMU() {
    navX.reset();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return navX.getVelocityZ() * (SwerveConstants.kGyroReversed ? -1.0 : 1.0);
    //return m_gyro.getRate(IMUAxis.kZ) * (SwerveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public float getPitch() {
    return navX.getPitch();
  }

  public float getRoll() {
    return navX.getRoll();
  }



    //---------------ODOMETRY METHODS --------------//
  public SwerveDriveOdometry getOdometry() {
    return driveOdometry;
  }

   /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d newPose) {
    //driveOdometry.resetPosition(getHeading(), getSwerveModulePos(), newPose);
    driveOdometry.resetPosition(
        Rotation2d.fromDegrees(getHeadingRadians()),
        getSwerveModulePos(),
        newPose);
  }

  public void updateOdometry() {
      driveOdometry.update(
        Rotation2d.fromDegrees(getHeadingRadians()),
        getSwerveModulePos()
    );
  }

  public Pose2d getPose2d() {
    return driveOdometry.getPoseMeters();
  }

  public Field2d getField() {
    return field;
  }

  public SwerveDriveKinematics getKinematics() {
    return driveKinematics;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return driveOdometry.getPoseMeters();
  }



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
    SmartDashboard.putNumber("turnRate", getTurnRate());

    //SmartDashboard.putData("Odometry Field", field);
  }





  @Override
  public void periodic() {
    updateOdometry();
    updateTelemetry();
  }



}
