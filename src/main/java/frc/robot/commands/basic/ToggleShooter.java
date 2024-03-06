// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.basic;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class ToggleShooter extends Command {
  /** Creates a new ToggleShooter. */

  private static Launcher launcher;
  private Supplier<Boolean> toggleOn;
  private Supplier<Boolean> toggleBrake;

  private boolean brakeToggle;
  private boolean revToggle;

  public ToggleShooter(Supplier<Boolean> toggleOn, Supplier<Boolean> toggleBrake) {
    this.toggleOn = toggleOn;
    this.toggleBrake = toggleBrake;
    brakeToggle = false;
    revToggle = false;

    launcher = Launcher.getInstance();
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launcher.launch(0);
    launcher.setCoast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (toggleBrake.get()) {
      brakeToggle = !brakeToggle;
    }

    if (toggleOn.get()) {
      revToggle = !revToggle;
    }

    if (revToggle) {
      launcher.launch(1);
    } else {
      launcher.launch(0);
    }

    if (brakeToggle) {
      launcher.setBrake();
    } else {
      launcher.setCoast();
    }

    SmartDashboard.putBoolean("brake toggle", brakeToggle);
    SmartDashboard.putBoolean("rev toggle", revToggle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcher.launch(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
