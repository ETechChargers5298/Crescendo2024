// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closed;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.MechConstants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;

public class TurnToApril extends PIDCommand {

  private Drivetrain drivetrain;
  private Camera cam;
  // private PIDController controller;
  /** Creates a new TurnToApril. */
  public TurnToApril() {
    super(
        // The controller that the command will use
        new PIDController(0.5, 0, 0),
        // This should return the measurement
        () -> Camera.getInstance().getY(),
        // This should return the setpoint (can also be a constant)
        () -> 0.0, //try to get robot to stop when y-value is 0
        // This uses the output
        output -> {
          // Use the output here
           Drivetrain.getInstance().setRotSpeed(-output);
        }
      );


    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = Drivetrain.getInstance();
    cam = Camera.getInstance();
    //addRequirements( drivetrain );
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.5);  //how far off in meters y-value can be for a good shotler.
  }

  // Called when the command is initially scheduled.
  // @Override
  // public void initialize() {
  //   drivetrain.drive(0, 0, 0);
  //   controller.reset();

  // }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double angle = cam.getY();
    // double angleRot = getController().calculate(angle, 0);
      drivetrain.setShutUpRotSpeedJoystick(true);
    if (!cam.hasTarget()) {
      drivetrain.stopDrive();
    } else {
      super.execute();
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    drivetrain.setShutUpRotSpeedJoystick(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
