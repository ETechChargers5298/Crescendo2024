package frc.robot.subsystems;

import org.photonvision.targeting.PhotonTrackedTarget;

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

  public PhotonTrackedTarget getDesiredTarget(int target) {
    return cam.getDesiredTarget(target);
  }

  public double getDesiredX(PhotonTrackedTarget target) {
    return cam.getDesiredX(target);
  }

  public double getX(){
    return cam.getX();
  }

  public double getDesiredY(PhotonTrackedTarget target) {
    return cam.getDesiredY(target);
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

  // public boolean isGreenZone(){
  //   if(isGreenZoneX() && isGreenZoneY() && isGreenZoneAngle()){
  //     return true;
  //   }
  //   return false;
  // }

  public boolean isGreenZone(){
    int blueSpeakerID = 7;
    int redSpeakerID = 3;


    if(getDesiredTarget(blueSpeakerID)!=null  || getDesiredTarget(redSpeakerID)!=null){
      return true;
    }
    return false;
  }
    public boolean isVioletZone(){
    int blueAmpID = 3;
    int redAmpID = 4;


    if(getDesiredTarget(blueAmpID)!=null  || getDesiredTarget(redAmpID)!=null){
      return true;
    }
    return false;
  }

  public boolean hasTarget() {
    return cam.hasTarget();
  }

  public String getAllianceColor() {
    return cam.getAllianceColor();
  }

  @Override
  public void periodic() {
    cam.update();
    SmartDashboard.putBoolean("hasTargetNew", cam.hasTarget());
    SmartDashboard.putNumber("X", getX());
    SmartDashboard.putNumber("Y", getY());
    SmartDashboard.putNumber("Desired X 7", getDesiredX(getDesiredTarget(7)));
    SmartDashboard.putNumber("Desired Y 7", getDesiredY(getDesiredTarget(7)));
    SmartDashboard.putNumber("Desired X 3", getDesiredX(getDesiredTarget(3)));
    SmartDashboard.putNumber("Desired Y 3", getDesiredY(getDesiredTarget(3)));
    SmartDashboard.putNumber("Z", cam.getRot());
    // SmartDashboard.putBoolean("isBlueZone", isBlueZone());
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
    } 
    // else if (isBlueZone()) {
    //   LEDStrip.request(SubsystemPriority.VISION, LEDStrip.IN_BLUE_ZONE);
    // }
  }
}