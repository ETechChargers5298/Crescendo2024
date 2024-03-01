// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.utils.LEDColors;

/** Add your docs here. */
public class LEDStrip extends SubsystemBase {

    private static Spark LED = new Spark(Ports.BLINKIN_PORT);

    //public static final double IntakeEat = LEDColors.HOT_PINK;
    public static final double HAVE_NOTE = LEDColors.ORANGE;
    public static final double IN_GREEN_ZONE = LEDColors.GREEN;
    public static final double IN_BLUE_ZONE = LEDColors.BLUE;
    public static final double DISABLED = LEDColors.RAINBOW_RAINBOW;
    public static final double ENABLED = LEDColors.OCEAN_COLOR_WAVES;
    public static final double AMP_ANGLE = LEDColors.VIOLET;
    public static final double SPEAKER_ANGLE = LEDColors.FOREST_BPM;
    public static final double CLIMBER_REACHED_MAX = LEDColors.WHITE;

    private static int topCurrentPriority = 0;

    private static double[] patternArray = new double[5];

    public enum SubsystemPriority {
        CLIMBING(4),
        ARM(3),
        VISION(2),
        NOTE(1),
        DEFAULT(0);
  
        private int priority;

        private SubsystemPriority(int priority) {
            this.priority = priority;
        }

        public int get() {
            return priority;
        }
    }

    public static void setPattern(double ledPattern) {
        LED.set(ledPattern);
    }

    public static void disable() {
        LED.stopMotor();
    }

    //
    public static void request(SubsystemPriority priority, double lightColor) {
        // update the top priority
        if (priority.get() > topCurrentPriority) {
            topCurrentPriority = priority.get();
        }
        // recording the request in the array
        patternArray[priority.get()] = lightColor;
    }

    public static void setStatus() {
        // turn light on for the top priority reqeust
        if (topCurrentPriority < patternArray.length) {
            setPattern(patternArray[topCurrentPriority]);
        }

        else {
            // Alliance alliance = DriverStation.getAlliance();
            // switch(alliance) {
            // case Red:
            // setPattern(ALLIANCE_RED_DEFAULT);
            // break;
            // case Blue:
            // setPattern(ALLIANCE_BLUE_DEFAULT);
            // break;
            // default:
            // setPattern(FIV_TOO_NEIN_EIGT);
            // break;
            // }
        }

        // resets top priority back to default
        topCurrentPriority = SubsystemPriority.DEFAULT.get();

        SmartDashboard.putNumber("Top Priority", topCurrentPriority);
        SmartDashboard.putNumber("LED Value", patternArray[topCurrentPriority]);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        request(SubsystemPriority.DEFAULT, ENABLED);
        setStatus();
    }

}
