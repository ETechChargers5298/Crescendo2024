// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closed;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToApril extends PIDCommand {

  private static Drivetrain drivetrain;
  private static Camera cam;

  /** Creates a new TurnToApril. */
  public TurnToApril() {
    super(
        // The controller that the command will use
        new PIDController(0.005, 0, 0),
        // This should return the measurement
        () -> drivetrain.getHeading().getDegrees(),
        // This should return the setpoint (can also be a constant)
        () -> drivetrain.getHeading().getDegrees() - cam.getX(),
        // This uses the output
        output -> {
          drivetrain.drive(0, 0, output);
          // Use the output here
        });

        drivetrain = Drivetrain.getInstance();
        cam = Camera.getInstance();

        addRequirements(drivetrain, cam);
        getController().setTolerance(0.5);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
