// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closed;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class ArmSetAngleApril extends ArmSetAngle {
  /** Creates a new ArmSetAngleApril. */
  public ArmSetAngleApril() {
    super(16.6 + 9.29 * Camera.getInstance().getX() + -0.645 * Math.pow(Camera.getInstance().getX(), 2));

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  // @Override
  // public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {}

  // // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {}

  // // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return false;
  // }
}