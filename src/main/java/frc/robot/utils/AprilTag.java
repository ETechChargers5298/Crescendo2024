package frc.robot.utils;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot;
import edu.wpi.first.math.util.Units;


public class AprilTag {

    private final int id;
    private final Pose3d blueLocation;

    public AprilTag(int id, Pose3d blueLocation) {
        this.id = id;
        this.blueLocation = blueLocation;
    }

    public int getID() {
        return id;
    }

    public Pose3d getBluePose(){
        return blueLocation;
    }

    public Pose3d getRedPose(){
        return transformToOppositeAlliance(blueLocation);
    }

    public Pose3d getLocation() {
        if (Robot.isBlue()) {
            return getBluePose();
        } else {
            return getRedPose();
        }
    }

    public static double FIELD_WIDTH = Units.inchesToMeters(323.25);
    public static double FIELD_LENGTH = Units.inchesToMeters(651.25);

    public static Pose3d transformToOppositeAlliance(Pose3d pose) {
        Pose3d rotated = pose.rotateBy(new Rotation3d(0, 0, Math.PI));

        return new Pose3d(
            rotated.getTranslation().plus(new Translation3d(FIELD_LENGTH, FIELD_WIDTH, 0)),
            rotated.getRotation());
    }


}
