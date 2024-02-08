// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.ModuleConfig;
import frc.robot.utils.PIDF;

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
    // public static final ModuleConfig SWERVE_FL = new ModuleConfig("FL", Ports.SWERVE_DRIVE_FL, Ports.SWERVE_TURN_FL, (2.9483314 + Math.PI/2) % (2 * Math.PI));
    // public static final ModuleConfig SWERVE_FR = new ModuleConfig("FR", Ports.SWERVE_DRIVE_FR, Ports.SWERVE_TURN_FR, 5.6096436);
    // public static final ModuleConfig SWERVE_BL = new ModuleConfig("BL", Ports.SWERVE_DRIVE_BL, Ports.SWERVE_TURN_BL, 0.6873395);
    // public static final ModuleConfig SWERVE_BR = new ModuleConfig("BR", Ports.SWERVE_DRIVE_BR, Ports.SWERVE_TURN_BR, (1.9551601 + Math.PI/2) % (2 * Math.PI));

    public static final ModuleConfig SWERVE_FL = new ModuleConfig("FL", Ports.SWERVE_DRIVE_FL, Ports.SWERVE_TURN_FL, 2.9483314);
    public static final ModuleConfig SWERVE_FR = new ModuleConfig("FR", Ports.SWERVE_DRIVE_FR, Ports.SWERVE_TURN_FR, 5.6096436);
    public static final ModuleConfig SWERVE_BL = new ModuleConfig("BL", Ports.SWERVE_DRIVE_BL, Ports.SWERVE_TURN_BL, 0.6873395);
    public static final ModuleConfig SWERVE_BR = new ModuleConfig("BR", Ports.SWERVE_DRIVE_BR, Ports.SWERVE_TURN_BR, 1.9551601);

    public static final double FL_ANGULAR_OFFSET = -Math.PI / 2;
    public static final double FR_ANGULAR_OFFSET = 0;
    public static final double BL_ANGULAR_OFFSET = Math.PI;
    public static final double BR_ANGULAR_OFFSET = Math.PI / 2;

    public static final PIDF TURN_PID = new PIDF(0.16, 0, 2 * Math.PI, -1, 1, true);
    public static final double ANGLE_THRESHOLD = Units.degreesToRadians(1);
    public static final  boolean TURN_INVERSION = true;
    public static final double TOP_ANGULAR_SPEED = 2 * 2 * Math.PI;

    public static final double TOP_SPEED = Units.feetToMeters(9.6);
    public static final double GEER_RATTIOLI = 5.08;

    public static final double TRACK_WIDTH = Units.inchesToMeters(25);
    public static final double WHEEL_BASE = Units.inchesToMeters(25);
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3);

    // public static final PIDConstants translationPID = new PIDConstants(0.05, 0, 0);
    // public static final PIDConstants rotationPID = new PIDConstants(0.08, 0, 0);

  }

  public static class CameraConstants {
    public static final String GRID_APRIL_CAM_NAME = "Arducam_OV9782_USB_Camera";
  }

  public static class MechConstants{

    public static final int INTAKE_MOTOR_PORT = 9;
    public static final int TOP_LAUNCHER_MOTOR_PORT = 10;
    public static final int BOTTOM_LAUNCHER_MOTOR_PORT = 11;

    public static final int DOWN_PIVOT_MOTOR_PORT = 10;
    public static final int UP_PIVOT_MOTOR_PORT = 13;
    
    public static final int CLIMB_REACH_MOTOR_PORT = 14;
    public static final int CLIMB_RETRACT_MOTOR_PORT = 15;
  }
    
  public static class Ports{
     //USB Ports
    public static final int DRIVER = 0;
    public static final int OPERATOR = 1;
  }

  public static class MotorSpeeds {
    public static final double LAUNCHER_SPEED = 1.0;
    public static final double INTAKE_SPEED = 1.0;
    public static final double ARM_PIVOT_SPEED = 1.0;
    
  }
}  
}

