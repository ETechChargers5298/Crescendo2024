package frc.robot.utils;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilCam {

    private PhotonCamera camera;
    private PhotonPipelineResult result;

    public AprilCam(String name) {
        this.camera = new PhotonCamera(name);
    }

    public void update() {
        this.result = camera.getLatestResult();
    }

    public boolean hasTarget() {
        return result.hasTargets();
    }

    public int getBestID(){
        PhotonTrackedTarget target = result.getBestTarget();
        if(target == null) {
            return -1;
        }
        return target.getFiducialId();   
    }
    

}
