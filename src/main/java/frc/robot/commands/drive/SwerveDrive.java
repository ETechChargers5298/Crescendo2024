// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

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

  /** Creates a new SwerveDrive. */
  public SwerveDrive(Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rotSpeed) {

    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rotSpeed = rotSpeed;
    xFilter = new SlewRateLimiter(1.5);
    yFilter = new SlewRateLimiter(1.5);

    drivetrain = Drivetrain.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.drive(0, 0, 0);
    drivetrain.resetIMU();
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
