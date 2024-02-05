package frc.robot.commands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.AprilCam;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class MoveToTarget extends Command{

    private Drivetrain drivetrain;
    private AprilCam camera;

    int targetID = 0;
    double X;
    double Y;
    double Z;

    /** Rotates the robot and drives to the best (nearest) tracked target, can be used for either 
    * april tags or retroreflective tape tracked by photonvision
   */
  public MoveToTarget() {
    drivetrain = Drivetrain.getInstance();
    camera = new AprilCam("aprilCam");
    //desiredDistanceToTarget = distanceToTarget;
    addRequirements(drivetrain);
  }


    @Override
    public void initialize(){
        //arm.stop();
    }

    @Override
    public void execute(){
        
        double xSpeed = 0.0;
        double zSpeed = 0.0;
        double rot = 0.0;

        if(camera.hasTarget()){
            X = camera.printX();
            Z = camera.printZ();

           int trackedTarget  = camera.getBestID();
            if(trackedTarget  == 7){
                if(Z>0.3 && Z < 100){
                    zSpeed = - 0.5;
                } else if (Z< -0.3) {
                    zSpeed = 0.5;
                }
                if(X>1.3 && X < 100){
                    xSpeed = 0.5;
                } else if(X<0.8){
                    xSpeed = -0.5;
                }

            }
        }
    }

    @Override
    public void end(boolean interrupted){
        //arm.stop();
    }

    @Override
    public boolean isFinished() {
      return false;
    }




    


    
}