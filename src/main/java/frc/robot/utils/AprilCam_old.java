package frc.robot.utils;

import org.photonvision.PhotonCamera;
//import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class AprilCam_old {

    private PhotonPipelineResult result;
    private PhotonTrackedTarget target;
    private PhotonCamera camera;
    
    public AprilCam_old(String cameraName) {
        camera = new PhotonCamera(cameraName);
        update();
    }

    public void update() {
        this.result = camera.getLatestResult();
        this.target = result.getBestTarget();
    }

    public boolean hasTargets(){
        return(result.hasTargets());

    }

    public double getYaw() {
        if(target == null) {
            return 500;
        }
        return target.getYaw();
    }

    public double getPitch() {

        if(target == null) {
            return 500;
        }
        return(target.getPitch());
    }

    public double getArea(){

        if(target == null){
            return 500;
        }
        return(target.getArea());
    }

    public Transform3d getCameraToTarget() {

        if(target == null){
           return new Transform3d(500, 500, 500, new Rotation3d());//careywashere
        }
        return target.getBestCameraToTarget();
    }
    
}
