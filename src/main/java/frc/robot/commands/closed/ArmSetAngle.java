package frc.robot.commands.closed;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.MechConstants;
import frc.robot.subsystems.Arm;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmSetAngle extends PIDCommand {
  /** Creates a new ArmSetAngle. */
  private static Arm arm;

  public ArmSetAngle(double desiredAngle) {
    super(
        // The controller that the command will use
        new PIDController(0.02, 0, 0),
        // This should return the measurement
        () -> arm.getPosition(),
        // This should return the setpoint (can also be a constant)
        () -> desiredAngle,
        // This uses the output
        output -> {
          // Use the output here
          arm.pivot(output * 0.6);
        });
    
    // Use addRequirements() here to declare subsystem dependencies.
    arm = Arm.getInstance();
    addRequirements(arm);
    
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(MechConstants.ARM_POSITION_TOLERANCE);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
