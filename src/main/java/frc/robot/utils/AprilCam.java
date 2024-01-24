package frc.robot.utils;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;

public class AprilCam {

    private PhotonPipelineResult result;
    private PhotonTrackedTarget target;
    private PhotonCamera camera;

    public AprilCam(String cameraName) {
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
        return(target.getYaw()); 
    }

    public double getPitch() {
        return(target.getPitch());
    }

    public double getArea(){
        return(target.getArea());
    }

    public Transform3d getCameraToTarget() {
        return target.getBestCameraToTarget();
    }
    
}
