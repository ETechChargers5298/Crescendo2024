// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closed;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DrivePID extends Command {

private Drivetrain drivetrain;
 private double distanceMeters, strafeMeters, angle;
 private PIDController distancePID, strafePID, anglePID;
 private double start;



  /** Creates a new DrivePID. */
  public DrivePID(double desiredDistance, double strafeMeters, double angle) {
    drivetrain = Drivetrain.getInstance();
    this.distancePID = new PIDController(1.0, 0.0, 0.0);

    start = drivetrain.getPose().getX();
    double targetPoint = start + desiredDistance;
    this.distancePID.setSetpoint(targetPoint);
    this.distancePID.setTolerance(0.05);

    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.drive(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double current = drivetrain.getPose().getX();

    double xSpeed = distancePID.calculate(current);
    drivetrain.drive(xSpeed, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distancePID.atSetpoint();
  }
}
