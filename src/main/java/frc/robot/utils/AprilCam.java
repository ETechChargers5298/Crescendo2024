package frc.robot.utils;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;

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

    public double printX(){
        PhotonTrackedTarget target = result.getBestTarget();
        if(target == null) {
            return 500.0;
        }
        Transform3d tea = target.getBestCameraToTarget();   
        return tea.getX();
    }
    
        public double printY(){
        PhotonTrackedTarget target = result.getBestTarget();
        if(target == null) {
            return 500.0;
        }
        Transform3d tea = target.getBestCameraToTarget();   
        return tea.getY();
    }

}
