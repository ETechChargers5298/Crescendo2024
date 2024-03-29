package frc.robot.commands.basic;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechConstants;
import frc.robot.subsystems.Launcher;


public class LauncherTake extends Command {

  private Launcher launcher;
  
  /** Creates a new IntakeEat object */
  public LauncherTake() {
    launcher = Launcher.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launcher.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launcher.take(MechConstants.LAUNCHER_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcher.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
