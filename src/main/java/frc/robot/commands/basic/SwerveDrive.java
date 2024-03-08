package frc.robot.commands.basic;


import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;


public class SwerveDrive extends Command {

  private Drivetrain drivetrain;
  private Supplier<Double> xSpeed;
  private Supplier<Double> ySpeed;
  private Supplier<Double> rotSpeed;
  private SlewRateLimiter xFilter;
  private SlewRateLimiter yFilter;
  private Supplier<Boolean> fieldReset;

  /** Creates a new SwerveDrive. */
  public SwerveDrive(Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rotSpeed, Supplier<Boolean> fieldReset) {

    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rotSpeed = rotSpeed;
    this.fieldReset = fieldReset;
    xFilter = new SlewRateLimiter(1.2);
    yFilter = new SlewRateLimiter(1.2);

    drivetrain = Drivetrain.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.drive(0, 0, 0);
    //drivetrain.resetIMU();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(
      xFilter.calculate(MathUtil.applyDeadband(xSpeed.get(), 0.1)), 
      yFilter.calculate(MathUtil.applyDeadband(ySpeed.get(), 0.1)),
      MathUtil.applyDeadband(rotSpeed.get(), 0.1), 
      drivetrain.getFieldCentric()
      );

      if(fieldReset.get()) {
        drivetrain.resetIMU();
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
