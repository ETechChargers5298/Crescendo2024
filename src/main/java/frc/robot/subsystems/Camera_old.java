package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;
import frc.robot.utils.AprilCam_old;


public class Camera_old extends SubsystemBase {
  
  private AprilCam_old gridAprilCam;
 
  private static Camera_old instance;
  
  /** Creates a new Camera. */
  
  public Camera_old() {
    gridAprilCam = new AprilCam_old(CameraConstants.GRID_APRIL_CAM_NAME);  
  }

  public AprilCam_old getGridAprilCam(){
    return gridAprilCam;
  }

  public static Camera_old getInstance() {
    if (instance == null) {
      instance = new Camera_old();
    }
    return instance;
  }
 


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Yaw", gridAprilCam.getYaw());
    SmartDashboard.putNumber("Pitch", gridAprilCam.getPitch());
    SmartDashboard.putNumber("Area", gridAprilCam.getArea());
    SmartDashboard.putBoolean("hasTarget", gridAprilCam.hasTargets());
  }
}