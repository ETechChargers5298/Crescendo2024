package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.AprilCam;


public class Camera extends SubsystemBase {

  private AprilCam cam;
  private static Camera instance;

  private Camera() {
    this.cam = new AprilCam(VisionConstants.APRIL_CAM_NAME);
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

  public double getX(){
    return cam.getX();
  }

  public double getY(){
    return cam.getY();
  }

  public double getZ(){
    return cam.getZ();
  }

  public boolean isGreenZoneX(){
    if(getX() <= VisionConstants.GREENZONE_MAX_X) {
        return true;
    }
    return false;
  }

  public boolean isGreenZoneY(){
    if(getY() <= VisionConstants.GREENZONE_MAX_Y 
    && getY() >= VisionConstants.GREENZONE_MIN_Y) {
        return true;
    }
    return false;
  }

  public boolean isGreenZoneAngle(){
    if(Math.abs(getZ()) <= VisionConstants.GREENZONE_MAX_ANGLE) {
        return true;
    }
    return false;
  }

  public boolean isGreenZone(){
    if(isGreenZoneX() && isGreenZoneY() && isGreenZoneAngle()){
      return true;
    }
    return false;
  }

  public boolean isBlueZone(){
    if(cam.getDesiredTarget()!=null){
      return true;
    }
    return false;
  }


  @Override
  public void periodic() {
    cam.update();
    SmartDashboard.putBoolean("hasTargetNew", cam.hasTarget());
    SmartDashboard.putNumber("BestID", cam.getBestID());
    SmartDashboard.putNumber("X", getX());
    SmartDashboard.putNumber("Y", getY());
    SmartDashboard.putNumber("Z", getZ());
    SmartDashboard.putBoolean("isBlueZone", isBlueZone());
    SmartDashboard.putBoolean("isGreenZone", isGreenZone());
  }
}