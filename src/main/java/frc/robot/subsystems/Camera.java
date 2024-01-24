package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;
import frc.robot.utils.AprilCam;


public class Camera extends SubsystemBase {
  
  private AprilCam gridAprilCam;
 
  private static Camera instance;
  
  /** Creates a new Camera. */
  
  public Camera() {
    
    gridAprilCam = new AprilCam(CameraConstants.GRID_APRIL_CAM_NAME);
  

  }

  public AprilCam getGridAprilCam(){
    return gridAprilCam;
  }

  public static Camera getInstance() {
    if (instance == null) {
      instance = new Camera();
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