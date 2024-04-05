package frc.robot.commands.basic;


import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;


public class SwerveDriveNew extends Command {

  private Drivetrain drivetrain;
  private Supplier<Double> xSpeed;
  private Supplier<Double> ySpeed;
  private Supplier<Double> rotSpeed;
  private SlewRateLimiter xFilter;
  private SlewRateLimiter yFilter;
  private Supplier<Boolean> fieldReset;

  private Supplier<Boolean> tta;
  private PIDController ttaController;
  private double ttaSpeed;
  private Camera cam;

  /** Creates a new SwerveDrive. */
  public SwerveDriveNew(Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rotSpeed, Supplier<Boolean> fieldReset, Supplier<Boolean> tta) {

    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rotSpeed = rotSpeed;
    this.fieldReset = fieldReset;
    this.tta = tta;
    xFilter = new SlewRateLimiter(1.2);
    yFilter = new SlewRateLimiter(1.2);

    cam = Camera.getInstance();

    ttaController = new PIDController(0.4, 0, 0);
    ttaController.setSetpoint(0);
    ttaController.setTolerance(0.01);

    //ttaSpeed = ttaController.calculate(cam.getY());

    drivetrain = Drivetrain.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
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
    drivetrain.setXSpeed(MathUtil.applyDeadband(xSpeed.get(), 0.1));
    drivetrain.setYSpeed(MathUtil.applyDeadband(ySpeed.get(), 0.1));

    if(tta.get() && cam.isGreenZone())
    {
      if(cam.getAllianceColor().equals("BLUE")) {
       ttaSpeed = ttaController.calculate(cam.getDesiredY(cam.getDesiredTarget(7)));
      } else if(cam.getAllianceColor().equals("RED")) {
        ttaSpeed = ttaController.calculate(cam.getDesiredY(cam.getDesiredTarget(3)));
      }
      drivetrain.setRotSpeed(-ttaSpeed);


    } else {
      drivetrain.setRotSpeed(MathUtil.applyDeadband(rotSpeed.get(), 0.1));

    }
      // xFilter.calculate(MathUtil.applyDeadband(xSpeed.get(), 0.1)), 
      // yFilter.calculate(MathUtil.applyDeadband(ySpeed.get(), 0.1)),
    
      if(fieldReset.get()) {
        drivetrain.resetIMU();
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
