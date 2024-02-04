package frc.robot.utils;

public class ModuleConfig {
    
    // fields for drive motor & turn motor
    public final int DRIVE_PORT;
    public final int TURN_PORT;

    // field for angle offset for turn motor to through-bore encoder
    public final double SENSOR_OFFSET;

    // field for offset related to how RevSwerveMax calibration tool sets wheel
    public final double ANGULAR_OFFSET;

    // name of module
    public final String NAME;

    public ModuleConfig(String name, int drivePort, int turnPort, double sensorOffset, double angularOffset) {

        // initializing fields
        NAME = name;
        DRIVE_PORT  = drivePort;
        TURN_PORT = turnPort;
        SENSOR_OFFSET = sensorOffset;
        ANGULAR_OFFSET = angularOffset;

    } 

}
