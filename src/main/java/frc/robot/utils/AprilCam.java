// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;


import frc.robot.Constants.VisionConstants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;
import java.util.Optional;


public class AprilCam {

    private PhotonCamera camera;
    private PhotonPipelineResult result;
    private PhotonTrackedTarget desiredTarget;

    private AprilTagFieldLayout fieldLayout;
    private Transform3d camOffset;
    private PhotonPoseEstimator photonPoseEstimator;
    private static AprilCam instance;
    

    //Primary Constructor
    public AprilCam(String name, Translation3d position, Rotation3d angle) {
        this.camera = new PhotonCamera(name);
        this.fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        this.camOffset = new Transform3d(position, angle);
        this.photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, camOffset);
    }

    //Simpler Constructor
    public AprilCam(String name) {
        this(name,new Translation3d(), new Rotation3d());
    }


    public static AprilCam getInstance() {
        if (instance == null) {
          instance = new AprilCam(VisionConstants.APRIL_CAM_NAME);
        }
        return instance;
      }

    public void update() {
        this.result = camera.getLatestResult();
    }

    
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {

        return photonPoseEstimator.update();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {

        // only matters for CLOSEST_TO_REFERENCE_POSE strategy
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

        return photonPoseEstimator.update();
    }

    public boolean hasTarget() {
        return result.hasTargets();
    }

    public boolean hasDesiredTarget(int desiredTargetID) {
        ///use the getDesiredTarget method to see if it returns null (not correct target) or not
        if (getDesiredTarget(desiredTargetID)!= null)
        {
            return true;
        }
        return false;
    }

    //method to get all the AprilTag targets the camera can see
    public List<PhotonTrackedTarget> getTargets(){
        return result.getTargets();
    }

    //method that returns a PhotonTrackedTarget object for the desired target
    public PhotonTrackedTarget getDesiredTarget(int desiredTargetID){

        //look at each target in the arraylist of targets
        for (PhotonTrackedTarget t: getTargets())
        {
            //look for the target with the desired ID
            if (t.getFiducialId() == desiredTargetID)
            {
                return t;
            }
            
        }

        //return null if you can't find the desiredTarget
        return null;
    }

    
    //method that gets the ID of the "best" target, generally not used by our robot
    public String getAllTargets(){
        MultiTargetPNPResult target = result.getMultiTagResult();

        return target.fiducialIDsUsed.toString();
    }



    public double getX(){
        PhotonTrackedTarget target = result.getBestTarget();
        if(target == null) {
            return Float.NaN;
        }
        Transform3d tea = target.getBestCameraToTarget();   
        return tea.getX();
    }
    
    public double getDesiredX(PhotonTrackedTarget target){
        if(target == null) {
            return Float.NaN;
        }
        Transform3d tea = target.getBestCameraToTarget();   
        return tea.getX();
    }

    public double getY(){
        PhotonTrackedTarget target = result.getBestTarget();
        if(target == null) {
            return Float.NaN;
        }
        Transform3d tea = target.getBestCameraToTarget();   
        return tea.getY();
    }

    public double getDesiredY(PhotonTrackedTarget target){
        if(target == null) {
            return Float.NaN;
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

    public double getRot(){
        PhotonTrackedTarget target = result.getBestTarget();
        if(target == null) {
            return 500.0;
        }
        Transform3d tea = target.getBestCameraToTarget();   
        return tea.getRotation().getAngle() / Math.PI * 180;
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
