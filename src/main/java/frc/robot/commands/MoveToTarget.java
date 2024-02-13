package frc.robot.commands;

import java.util.Optional;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.AprilCam;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;

public class MoveToTarget extends Command{

    private Drivetrain drivetrain;
    private Camera cam;

    int desiredTargetID = 7;
    double X;
    double Y;
    double Z;

    //speed variables
    double xSpeed = 0.0;
    double ySpeed = 0.0;
    double rot = 0.0;

    /** Rotates the robot and drives to the best (nearest) tracked target, can be used for either 
    * april tags or retroreflective tape tracked by photonvision
   */
  public MoveToTarget() {
    drivetrain = Drivetrain.getInstance();
    cam = Camera.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(
      drivetrain,
      cam
      );

    // add code to change the desiredTargetID based on the alliance color
    
    String color = cam.getCam().getAllianceColor();
    if (color.equals("RED") )
    {
        desiredTargetID = 4;
    }
    else if (color.equals("BLUE"))
    {
        desiredTargetID = 7;
    }
    else
    {
        desiredTargetID = 7;
    }

  }


    @Override
    public void initialize(){
        drivetrain.drive(0, 0, 0);
        drivetrain.resetIMU();

        //X stands for distance forward and backward from target (+ looks forward)(Meters)
        //Y stands for distance left and right from target(+ look right)(- look left)(Meters)
        X = cam.getCam().getX();
        Y = cam.getCam().getY();
    }

    @Override
    public void execute(){
        if(cam.getCam().hasDesiredTarget()){

            //if we're far from target, the move forward                        
            if(X > VisionConstants.GREENZONE_MAX_X ){   ///X = 1.3
                xSpeed = 0.5;
            }
            //if we're too close to the target, move backward
            else if(X < VisionConstants.GREENZONE_MIN_X){    ///X = 0.8
                xSpeed = -0.5;
            }

            //if target is to the left of our robot, strafe right
            if(Y>VisionConstants.GREENZONE_MAX_Y){   ///Y = 0.3
                ySpeed = - 0.5;
            } 
            
            //if target is to the right of our robot, strafe left
            else if (Y< VisionConstants.GREENZONE_MIN_Y) {   ///Y = -0.3
                ySpeed = 0.5;
            }

        }

        //add some feedback (lights?) if we dont' see the target when we THINK we see the target
        // else{

        // }
        drivetrain.drive(xSpeed, ySpeed, rot);
    }


    @Override
    public void end(boolean interrupted){
        drivetrain.drive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
        //return !camera.hasDesiredTarget();    //will stop the Command if no target is seen
    }




    


    
}