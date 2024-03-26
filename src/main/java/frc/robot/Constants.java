// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.utils.ModuleConfig;
import frc.robot.utils.PIDF;
import frc.robot.Ports;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

//import edu.wpi.first.apriltag.AprilTag;
import frc.robot.utils.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class SwerveConstants {
    ///(Old Robot)
    //Sensor Offsets for the radian difference between the physical sensor orientation and the calibrated swerve direction
    public static final double FL_SENSOR_OFFSET = 5.1247236; //from REV Hardware Client
    public static final double FR_SENSOR_OFFSET = 4.7080523; //from REV Hardware Client
    public static final double BR_SENSOR_OFFSET = 2.9550133; //from REV Hardware Client
    public static final double BL_SENSOR_OFFSET = 2.8204095; //from REV Hardware Client

    ///(New Robot)(Put new numbers)
    // public static final double FL_SENSOR_OFFSET = 0.7848113; //from REV Hardware Client
    // public static final double FR_SENSOR_OFFSET = 4.7082246; //from REV Hardware Client
    // public static final double BR_SENSOR_OFFSET = 1.9979531; //from REV Hardware Client
    // public static final double BL_SENSOR_OFFSET = 1.6861502; //from REV Hardware Client


    //Angular Offsets for the radian difference between the calibrated swerve and desired forward direction
    // public static final double FL_ANGULAR_OFFSET = -Math.PI/2; //Math.PI / 2; //-Math.PI / 2;
    // public static final double FR_ANGULAR_OFFSET = -Math.PI/2;
    // public static final double BR_ANGULAR_OFFSET = 3 * Math.PI/4; //Math.PI / 2;
    // public static final double BL_ANGULAR_OFFSET = Math.PI/2; //Math.PI;

    ///(New Robot)(Put new numbers)
    //Angular Offsets for the radian difference between the calibrated swerve and desired forward direction
    public static final double FL_ANGULAR_OFFSET = -Math.PI/2; //Math.PI / 2; //-Math.PI / 2;
    public static final double FR_ANGULAR_OFFSET = 0;
    public static final double BR_ANGULAR_OFFSET = -Math.PI / 2; //Math.PI / 2;
    public static final double BL_ANGULAR_OFFSET = 0; //Math.PI;

    //Constructor to hold all of the data to configure a SwerveModule
    public static final ModuleConfig SWERVE_FL = new ModuleConfig("FL", Ports.SWERVE_DRIVE_FL, Ports.SWERVE_TURN_FL, FL_SENSOR_OFFSET, FL_ANGULAR_OFFSET, false);//2.9483314  +Math.PI /2);
    public static final ModuleConfig SWERVE_FR = new ModuleConfig("FR", Ports.SWERVE_DRIVE_FR, Ports.SWERVE_TURN_FR, FR_SENSOR_OFFSET, FR_ANGULAR_OFFSET, false);
    public static final ModuleConfig SWERVE_BL = new ModuleConfig("BL", Ports.SWERVE_DRIVE_BL, Ports.SWERVE_TURN_BL, BL_SENSOR_OFFSET, BL_ANGULAR_OFFSET, true); //0.6873395
    public static final ModuleConfig SWERVE_BR = new ModuleConfig("BR", Ports.SWERVE_DRIVE_BR, Ports.SWERVE_TURN_BR, BR_SENSOR_OFFSET, BR_ANGULAR_OFFSET, true);

    // Chassis configuration
    public static final double TRACK_WIDTH = Units.inchesToMeters(25);

    // Distance between centers of right and left wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(25);

    public static final double DISTANCE_TO_CENTER = Math.sqrt(Math.pow(WHEEL_BASE/2, 2) + Math.pow(WHEEL_BASE/2, 2));

    // Diamter of the REV Swerve wheels in inches
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3);
    public static final double FREE_SPIN_METER = 6.36;


    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));


    //TURN PID CONSTANTS
    //public static final PIDF TURN_PID = new PIDF(0.16, 0, 2 * Math.PI, -1, 1, true);
    public static final double ANGLE_THRESHOLD = Units.degreesToRadians(5);
    public static final boolean TURN_INVERSION = true;

    // Driving Parameters - max speeds allowed, not capable
    public static final double TOP_SPEED = Units.feetToMeters(2 * 4.8); //9.6
    public static final double TOP_ANGULAR_SPEED = 2 * Math.PI;
    public static final double GEER_RATTIOLI = 3.56;


    //Slew stuff from Rev
    public static final double kDirectionSlewRate = 1; // radians per second
    public static final double kMagnitudeSlewRate = 1.4; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 1; // percent per second (1 = 100%)

    public static final boolean kGyroReversed = false;

    // public static final PIDConstants translationPID = new PIDConstants(0.05, 0, 0);
    // public static final PIDConstants rotationPID = new PIDConstants(0.08, 0, 0);

  }


  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kNeoFreeSpeedRpm = 5676;
    public static final double kDrivingMotorFreeSpeedRps = kNeoFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = -Math.PI; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = Math.PI; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(5.0, 0, 0), // Translation constants 
      new PIDConstants(5.0, 0, 0), // Rotation constants 
      SwerveConstants.FREE_SPIN_METER, 
      SwerveConstants.DISTANCE_TO_CENTER, // Drive base radius (distance from center to furthest module) 
      new ReplanningConfig()
    );
  }


  public static class MechConstants{

    
    public static final int ENCODER_TICKS = 8192; //Counts per Revolution

    //climber encoders
    // public static final double RIGHT_CLIMB_OFFSET = 0.340;
    // public static final double LEFT_CLIMB_OFFSET = 0.30926;


    //Climber Heights
    public static final double MAX_CLIMB_RIGHT = 120.0;
    public static final double BASE_CLIMB_RIGHT = 0.0;
    public static final double MAX_CLIMB_LEFT = 120.0;
    public static final double BASE_CLIMB_LEFT = 0.0;

    //Mech Motor Speeds for Buttons
    public static final double INTAKE_SPEED = 0.6;
    public static final double LAUNCHER_SPEED = 1.0;
    public static final double ARM_PIVOT_SPEED = 1.0;
    public static final double CLIMBER_SPEED = 0.3;

    //Arm Angles    
    public static final double START_ANGLE = 94;
    public static final double FLOOR_ANGLE = 0.0;
    public static final double LAUNCH_ANGLE = 23.7;
    public static final double AMP_ANGLE = 106;
    public static final double ARM_POSITION_TOLERANCE = 1.0;
    public static final double ARM_OFFSET = 355.4097962
    ; 
  }


  public static class VisionConstants{

    //GreenZone boundaries
    public static final double GREENZONE_MAX_X = 4.0; 
    public static final double GREENZONE_MIN_X = 0.8;
    public static final double GREENZONE_MAX_Y = 0.3;
    public static final double GREENZONE_MIN_Y = -0.3;
    public static final double GREENZONE_MAX_ANGLE = 15.0;

    //GoodLaunch boundaries
    public static final double LAUNCH_ANGLE_TOLERANCE = 2.0;
    public static final double AMP_ANGLE_TOLERANCE = 5.0;
    
    //Good Launch 2nd Order Equation Co-efficients
    public static final double kC = -11;
    public static final double kB = 27.5;
    public static final double kA = -3.06;    
    
    //Distance to Angle Constants
    public static final double DEGREES_PER_METER_SLOPE = 10.0;
    public static final double DEGREES_Y_INTERCEPT = -2.0;

    //Camera Name
    public static final String APRIL_CAM_NAME = "Arducam_OV9782_USB_Camera";
  }


  public class FieldConstants{
    
    //AprilTag constants
    public static final AprilTag APRILTAGS[] = {
      new AprilTag(1,  new Pose3d(new Translation3d(Units.inchesToMeters(593.68), Units.inchesToMeters(9.68), Units.inchesToMeters(53.38)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(120)))),
      new AprilTag(2,  new Pose3d(new Translation3d(Units.inchesToMeters(637.21), Units.inchesToMeters(34.79), Units.inchesToMeters(53.38)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(120)))),
      new AprilTag(3,  new Pose3d(new Translation3d(Units.inchesToMeters(652.73), Units.inchesToMeters(196.17), Units.inchesToMeters(57.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
      new AprilTag(4,  new Pose3d(new Translation3d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), Units.inchesToMeters(57.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
      new AprilTag(5,  new Pose3d(new Translation3d(Units.inchesToMeters(578.77), Units.inchesToMeters(323.0), Units.inchesToMeters(53.38)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(270)))),
      new AprilTag(6,  new Pose3d(new Translation3d(Units.inchesToMeters(72.5), Units.inchesToMeters(323.0), Units.inchesToMeters(53.38)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(270)))),
      new AprilTag(7,  new Pose3d(new Translation3d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42), Units.inchesToMeters(57.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
      new AprilTag(8,  new Pose3d(new Translation3d(Units.inchesToMeters(-1.5), Units.inchesToMeters(196.17), Units.inchesToMeters(57.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
      new AprilTag(9,  new Pose3d(new Translation3d(Units.inchesToMeters(14.02), Units.inchesToMeters(34.79), Units.inchesToMeters(53.38)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(60)))),
      new AprilTag(10, new Pose3d(new Translation3d(Units.inchesToMeters(57.54), Units.inchesToMeters(9.68), Units.inchesToMeters(53.38)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(60)))),
      new AprilTag(11, new Pose3d(new Translation3d(Units.inchesToMeters(468.69), Units.inchesToMeters(146.19), Units.inchesToMeters(52.0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(300)))),
      new AprilTag(12, new Pose3d(new Translation3d(Units.inchesToMeters(468.69), Units.inchesToMeters(177.10), Units.inchesToMeters(52.0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(60)))),
      new AprilTag(13, new Pose3d(new Translation3d(Units.inchesToMeters(441.74), Units.inchesToMeters(161.62), Units.inchesToMeters(52.0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
      new AprilTag(14, new Pose3d(new Translation3d(Units.inchesToMeters(209.48), Units.inchesToMeters(161.62), Units.inchesToMeters(52.0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
      new AprilTag(15, new Pose3d(new Translation3d(Units.inchesToMeters(182.73), Units.inchesToMeters(177.10), Units.inchesToMeters(52.0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(120)))),
      new AprilTag(16, new Pose3d(new Translation3d(Units.inchesToMeters(182.73), Units.inchesToMeters(146.19), Units.inchesToMeters(52.0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(240)))),
    };

    //enumerations for each AprilTag
    enum NamedTags {
      BLUE_SOURCE_RIGHT,
      BLUE_SOURCE_LEFT,
      RED_SPEAKER_OFFSET,
      RED_SPEAKER,
      RED_AMP,
      BLUE_AMP,
      BLUE_SPEAKER,
      BLUE_SPEAKER_OFFSET,
      RED_SOURCE_RIGHT,
      RED_SOURCE_LEFT,
      RED_STAGE_LEFT,
      RED_STAGE_RIGHT,
      RED_STAGE_FAR,
      BLUE_STAGE_FAR,
      BLUE_STAGE_LEFT,
      BLUE_STAGE_RIGHT;

      public final AprilTag tag;

      public int getID() {
          return tag.getID();
      }

      public Pose3d getLocation() {
          return tag.getLocation();
      }

      private NamedTags() {
          tag = APRILTAGS[ordinal()];
      }
    }
  
  }

}  

