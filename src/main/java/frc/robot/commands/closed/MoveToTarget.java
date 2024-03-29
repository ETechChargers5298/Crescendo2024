package frc.robot.commands.closed;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;


public class MoveToTarget extends Command {

  private Drivetrain drivetrain;
  private Camera cam;


  /** Creates a new Forward. */
  public MoveToTarget() {
    drivetrain = Drivetrain.getInstance();
    cam = Camera.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(
      drivetrain,
      cam
      );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.stopDrive();
    //drivetrain.resetIMU();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(cam.getCam().getX() > Constants.VisionConstants.GREENZONE_MAX_X && cam.getCam().getY() > Constants.VisionConstants.GREENZONE_MAX_Y) {
      drivetrain.setDrive(0.8, 0.8, 0);
    }
    else if(cam.getCam().getX() > Constants.VisionConstants.GREENZONE_MAX_X && cam.getCam().getY() <= Constants.VisionConstants.GREENZONE_MAX_Y) {
      drivetrain.setDrive(0.8, 0, 0);
    }
    else {
        drivetrain.stopDrive();
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}