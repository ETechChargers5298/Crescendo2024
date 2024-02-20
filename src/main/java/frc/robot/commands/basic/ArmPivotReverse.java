package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechConstants;
import frc.robot.subsystems.Arm;

public class ArmPivotReverse extends Command {

  /** Creates a new ArmPivotReverse. */
  private Arm arm;

  public ArmPivotReverse() {
    arm = Arm.getInstance();
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.pivot(-MechConstants.ARM_PIVOT_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
