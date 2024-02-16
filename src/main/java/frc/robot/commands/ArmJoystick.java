// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmJoystick extends Command {

  private Arm arm;

  private Supplier<Double> speedX;
  
  /** Creates a new LauncherJoystick. */
  public ArmJoystick(Supplier<Double> speedX) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speedX = speedX;
    
    arm = Arm.getInstance();
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.pivot(0);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        arm.pivot(speedX.get());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

        arm.pivot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
