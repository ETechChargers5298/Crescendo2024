package frc.robot.commands.basic;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;


public class IntakeNoteStop extends Command {

  private static Intake intake;
  private boolean noteFound;

  /** Creates a new IntakeEat. */
  public IntakeNoteStop() {
    intake = Intake.getInstance();
    noteFound = false;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.stop();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Intake.isNoteFound) {
      intake.stop();
    } else {
      intake.eat();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Intake.isNoteFound;
  }
}

