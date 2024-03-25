package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.LEDStrip.SubsystemPriority;
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
    int blueSpeakerID = 1;
    int redSpeakerID = 2;


    if(cam.getDesiredTarget(blueSpeakerID)!=null  || cam.getDesiredTarget(redSpeakerID)!=null){
      return true;
    }
    return false;
  }
    public boolean isVioletZone(){
    int blueAmpID = 3;
    int redAmpID = 4;


    if(cam.getDesiredTarget(blueAmpID)!=null  || cam.getDesiredTarget(redAmpID)!=null){
      return true;
    }
    return false;
  }

  public boolean hasTarget() {
    return cam.hasTarget();
  }

  @Override
  public void periodic() {
    cam.update();
    SmartDashboard.putBoolean("hasTargetNew", cam.hasTarget());
    SmartDashboard.putNumber("X", getX());
    SmartDashboard.putNumber("Y", getY());
    SmartDashboard.putNumber("Z", cam.getRot());
    SmartDashboard.putBoolean("isBlueZone", isBlueZone());
    SmartDashboard.putBoolean("isGreenZone", isGreenZone());
    SmartDashboard.putBoolean("isVioletZone", isVioletZone());

    SmartDashboard.putNumber("desired angle", Drivetrain.getInstance().getHeading().getDegrees() + getZ());

    //SmartDashboard.putData("All ID's", (Sendable) cam.getAllTargets());
    for (int i = 0; i < cam.getTargets().size(); i++) {
      SmartDashboard.putString("id" + i, cam.getTargets().get(i).toString());
    }
    //SmartDashboard.putString("All ID's", cam.getTargets());
    if (isGreenZone()) {
      LEDStrip.request(SubsystemPriority.VISION, LEDStrip.IN_GREEN_ZONE);
    } else if (isBlueZone()) {
      LEDStrip.request(SubsystemPriority.VISION, LEDStrip.IN_BLUE_ZONE);
    }
  }
}