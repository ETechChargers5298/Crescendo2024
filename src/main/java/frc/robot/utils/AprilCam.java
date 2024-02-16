// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
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

public class AprilCam {
  /** Creates a new AprilCam. */
  private PhotonCamera camera;
    private PhotonPipelineResult result;
    PhotonTrackedTarget desiredTarget;
    private int desiredTargetID;
    private AprilTagFieldLayout fieldLayout;
    private Transform3d camOffset;
    private PhotonPoseEstimator photonPoseEstimator;
    private static AprilCam instance;
    

    public AprilCam(String name) {
        this.camera = new PhotonCamera(name);
        this.fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        this.camOffset = new Transform3d(new Translation3d(), new Rotation3d());
        this.photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, camOffset);
    }

    public AprilCam(String name, Translation3d position, Rotation3d angle) {
        this.camera = new PhotonCamera(name);
        this.fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        this.camOffset = new Transform3d(position, angle);
        this.photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, camOffset);
    }
    
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {

        return photonPoseEstimator.update();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {

        // only matters for CLOSEST_TO_REFERENCE_POSE strategy
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

        return photonPoseEstimator.update();
    }

    public static AprilCam getInstance() {
        if (instance == null) {
          instance = new AprilCam(Constants.CameraConstants.GRID_APRIL_CAM_NAME);
        }
        return instance;
      }

    public void update() {
        this.result = camera.getLatestResult();
    }

    public boolean hasTarget() {
        return result.hasTargets();
    }

    public boolean hasDesiredTarget() {
        ///use the getDesiredTarget method to see if it returns null (not correct target) or not
        if (getDesiredTarget()!= null)
        {
            return true;
        }
        return false;
    }

    public List<PhotonTrackedTarget> getTargets(){
        return result.getTargets();
    }

    public PhotonTrackedTarget getDesiredTarget(){
       // PhotonTrackedTarget target = result.getBestTarget();
        //get the arraylist of targets
        for (PhotonTrackedTarget t: getTargets())
        {
            if (t.getFiducialId() == this.desiredTargetID)
            {
                return t;
            }
            /////
        }
        return null;
    }

    
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
