// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closed;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DrivePID extends Command {

private Drivetrain drivetrain;
 private double desiredDistanceMeters, desiredStrafeMeters, desiredAngleDegrees;
 private PIDController distancePID, strafePID, anglePID;
 private double startDistance;



  /** Creates a new DrivePID. */
  public DrivePID(double desiredDistanceMeters, double desiredStrafeMeters, double desiredAngleDegrees) {
    
    //Instantiates the drivetrain object
    drivetrain = Drivetrain.getInstance();

    //saves desired pose values
    this.desiredDistanceMeters = desiredDistanceMeters;
    this.desiredStrafeMeters = desiredStrafeMeters;
    this.desiredAngleDegrees = desiredAngleDegrees;

    //setup for Distance PID Controller
    this.distancePID = new PIDController(1.0, 0.0, 0.0);
    startDistance = drivetrain.getPose().getX();
    double distanceSetPoint = startDistance + desiredDistanceMeters;
    this.distancePID.setSetpoint(distanceSetPoint);
    this.distancePID.setTolerance(0.05);

    //TODO: setup for Strafe PID Controller


    //TODO: setup for Angle PID Controller



    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.drive(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //calculating the distance
    double currentDistance = drivetrain.getPose().getX();
    double xSpeed = distancePID.calculate(currentDistance);

    //TODO: calculating the strafe


    //TODO: calculating the angle

    //calling drive function
    drivetrain.drive(xSpeed, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return distancePID.atSetpoint();
  }
}
