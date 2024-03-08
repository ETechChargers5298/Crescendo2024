// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closed;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;

public class TurnToApril extends Command {

  private Drivetrain drivetrain;
  private Camera cam;
  private PIDController controller;
  /** Creates a new TurnToApril. */
  public TurnToApril() {
    drivetrain = Drivetrain.getInstance();
    cam = Camera.getInstance();
    controller = new PIDController(0.0001, 0, 0);
    controller.setSetpoint(0);
    controller.setTolerance(3.0);

    addRequirements(
      drivetrain,
      cam
    );

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //drivetrain.drive(0, 0, 0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = cam.getY();
    double angleRot = controller.calculate(angle);

    // if(Camera.getInstance().getCam().hasTarget()){}
    drivetrain.drive(0, 0, angleRot);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0,0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
