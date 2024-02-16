package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.AprilCam;


public class Camera extends SubsystemBase {

  private AprilCam cam;
  private static Camera instance;

  private Camera() {
    this.cam = new AprilCam(CameraConstants.GRID_APRIL_CAM_NAME);
    cam.update();
  }

  public static Camera getInstance() {
    if (instance == null) {
      instance = new Camera();
    }
    return instance;
  }

  public AprilCam getCam() {
    return cam;
  }

  @Override
  public void periodic() {
    cam.update();
    SmartDashboard.putBoolean("hasTargetNew", cam.hasTarget());
    SmartDashboard.putNumber("BestID", cam.getBestID());
    SmartDashboard.putNumber("X", cam.getX());
    SmartDashboard.putNumber("Y", cam.getY());
    SmartDashboard.putNumber("Z", cam.getZ());
  }
}