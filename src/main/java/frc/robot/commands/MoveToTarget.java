package frc.robot.commands;

import java.util.Optional;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.AprilCam;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class MoveToTarget extends Command{

    private Drivetrain drivetrain;
    private AprilCam camera;

    int desiredTargetID = 7;
    double X;
    double Y;
    double Z;

    //speed variables
    double xSpeed = 0.0;
    double zSpeed = 0.0;
    double rot = 0.0;

    /** Rotates the robot and drives to the best (nearest) tracked target, can be used for either 
    * april tags or retroreflective tape tracked by photonvision
   */
  public MoveToTarget() {
    drivetrain = Drivetrain.getInstance();
    camera = new AprilCam("aprilCam");
    //desiredDistanceToTarget = distanceToTarget;
    addRequirements(drivetrain);

    //add code to change the desiredTargetID based on the alliance color
    //camera.getAllianceColor()

  }


    @Override
    public void initialize(){
        //drivetrain.stop();
    }

    @Override
    public void execute(){
        
        //check if we see the desired target on the screen
        if(camera.hasDesiredTarget()){

            X = camera.getX();
            Z = camera.getZ();

            //if we're far from target, the move forward                        
            if(X>1.3 && X < 100){
                xSpeed = 0.5;
            }
            //if we're too close to the target, move backward
            else if(X<0.8){
                xSpeed = -0.5;
            }

            //if target is to the left of our robot, strafe right
            if(Z>0.3){
                zSpeed = - 0.5;
            } 
            
            //if target is to the right of our robot, strafe left
            else if (Z< -0.3) {
                zSpeed = 0.5;
            }

        }

        //add some feedback (lights?) if we dont' see the target when we THINK we see the target
        else{

        }

    }


    @Override
    public void end(boolean interrupted){
        //drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
        //return !camera.hasDesiredTarget();    //will stop the Command if no target is seen
    }




    


    
}