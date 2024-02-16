//make changes to this



package frc.robot.commands.closed;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ComplexIntake extends Command {
  private static Intake intake;
  /** Creates a new IntakeEat. */
  public ComplexIntake() {
    intake = Intake.getInstance();
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
    intake.eat();
    if(intake.checkNoteFound()){
        intake.spitt();
    }
    //if sees the note it spitts for 0.1 second
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

