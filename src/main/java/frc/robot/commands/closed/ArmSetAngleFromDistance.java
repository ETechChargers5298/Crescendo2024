package frc.robot.commands.closed;


import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Camera;
import edu.wpi.first.wpilibj2.command.Command;


public class ArmSetAngleFromDistance extends Command {
  
    private Camera camera;
    private static Arm arm;
    double desiredAnglefromDistance;

    ///Set everything up
    public ArmSetAngleFromDistance()
    {
        camera = Camera.getInstance();
        arm = Arm.getInstance();

        addRequirements(arm, camera);
        double distanceInGreenZone = camera.getCam().getX();

        desiredAnglefromDistance = VisionConstants.DEGREES_PER_METER_SLOPE *distanceInGreenZone + VisionConstants.DEGREES_Y_INTERCEPT; ///Y = mx+b

    }
    //Make a constant inside VIsionConstant

    @Override
    public void initialize(){
        arm.pivot(0);
    }

    @Override
    public void execute(){
        // if (MoveToTarget.isFinished() == false)
        // {
        //     arm.pivot(desiredAnglefromDistance);
        // }
        
    }

    @Override
    public void end(boolean interrupted) {
      arm.pivot(0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }




    








}

