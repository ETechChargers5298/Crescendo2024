// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.basic;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechConstants;
import frc.robot.subsystems.Launcher;


public class LauncherShoot extends Command {
  private Launcher launch;
  /** Creates a new IntakeEat. */
  public LauncherShoot() {
    launch = Launcher.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(launch);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launch.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launch.launch(MechConstants.LAUNCHER_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launch.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
