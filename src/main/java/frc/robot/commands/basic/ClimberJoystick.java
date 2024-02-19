// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.basic;


import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;


public class ClimberJoystick extends Command {

  private Climber climber;
  private Supplier<Double> speed;
  

  /** Creates a new ClimberJoystick. */
  public ClimberJoystick( Supplier<Double> speed) {
    this.speed = speed;
    climber = Climber.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }


// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.move(speed.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
}
