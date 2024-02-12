// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AprilCam;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;

public class Forward extends Command {

  private Drivetrain drivetrain;
  private Camera cam;


  /** Creates a new Forward. */
  public Forward() {
    drivetrain = Drivetrain.getInstance();
    cam = Camera.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(
      drivetrain,
      cam
      );
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
    if(cam.getCam().getX() > Constants.VisionConstants.GREENZONE_MAX_X) {
      drivetrain.drive(0.1, 0, 0);
    }
    else {
      drivetrain.drive(-0.1, 0, 0);
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
