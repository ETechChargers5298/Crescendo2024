// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closed;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class ArmSetAngleApril extends Command {
  private Arm arm;
  private PIDController controller;

  /** Creates a new ArmSetAngleApril. */
  public ArmSetAngleApril() {
    arm = Arm.getInstance();
    controller = new PIDController(0.06, 0, 0);
    controller.setSetpoint(arm.getArmAprilAngle() + 2);
    controller.setTolerance(0.4);

    // Use addRequirements() here to declare subsystem dependencies.
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
    if(Camera.getInstance().getCam().hasTarget()){
      controller.setSetpoint(arm.getArmAprilAngle() + 2);
    } else {
      controller.setSetpoint(arm.getAngle());
    }
    
    double currentAngle = arm.getAngle();
    double angleSpeed = controller.calculate(currentAngle);
    arm.pivot(angleSpeed * 0.6);

    
  }

  // // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.pivot(0);
  }

  // // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
  
}
