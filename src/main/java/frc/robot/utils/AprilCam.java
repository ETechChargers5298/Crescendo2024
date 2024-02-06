package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AprilCam {

    private PhotonCamera camera;
    private PhotonPipelineResult result;
    PhotonTrackedTarget desiredTarget;
    private int desiredTargetID;

    public AprilCam(String name) {
        this.camera = new PhotonCamera(name);
    }

    public void update() {
        this.result = camera.getLatestResult();
    }

    public boolean hasTarget() {
        return result.hasTargets();
    }

    public boolean hasDesiredTarget() {
        //use the getDesiredTarget method to see if it returns null (not correct target) or not


    }

    public List<PhotonTrackedTarget> getTargets(){
        return result.getTargets();
    }

    public PhotonTrackedTarget getDesiredTarget(int desiredID){
        PhotonTrackedTarget target = result.getBestTarget();
        //get the arraylist of targets

        //look throught the list for the one that has the desiredID


        //return the desired target


        //if you don't find the desired target, then return null
        
        // if(target == null) {
        //     return -1;
        // }

    }

    // public int getDesiredID(){

    //     return target.getFiducialId();   
    // }
    
    public int getBestID(){
        PhotonTrackedTarget target = result.getBestTarget();
        if(target == null) {
            return -1;
        }
        return target.getFiducialId();   
    }



    public double getX(){
        PhotonTrackedTarget target = result.getBestTarget();
        if(target == null) {
            return 500.0;
        }
        Transform3d tea = target.getBestCameraToTarget();   
        return tea.getX();
    }
    
    public double getY(){
        PhotonTrackedTarget target = result.getBestTarget();
        if(target == null) {
            return 500.0;
        }
        Transform3d tea = target.getBestCameraToTarget();   
        return tea.getY();
    }
    public double getZ(){
        PhotonTrackedTarget target = result.getBestTarget();
        if(target == null) {
            return 500.0;
        }
        Transform3d tea = target.getBestCameraToTarget();   
        return tea.getZ();
    }


    public String getAllianceColor(){

        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                return "RED";
            }
            if (ally.get() == Alliance.Blue) {
                return "BLUE";
            }
        }
        return "NONE";

    }


}
