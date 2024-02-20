// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closed;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DrivePID extends Command {

  private Drivetrain drivetrain;
  private double desiredDistanceMeters; 
  private double desiredStrafeMeters;
  private double desiredAngleDegrees;
  private PIDController distancePID, strafePID, anglePID;
  private double startDistance;
  private double startStrafe;
  private double startAngle;


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
    this.strafePID = new PIDController(0.5, 0.0, 0.0);
    startStrafe = drivetrain.getPose().getY();
    double strafeSetPoint = startStrafe + desiredStrafeMeters;
    this.strafePID.setSetpoint(strafeSetPoint);
    this.strafePID.setTolerance(0.1);

    //TODO: setup for Angle PID Controller
    this.anglePID = new PIDController(0.005, 0.0, 0.0);
    startAngle = drivetrain.getPose().getRotation().getDegrees();
    double angleSetPoint = startAngle + desiredAngleDegrees;
    this.strafePID.setSetpoint(angleSetPoint);
    this.anglePID.setTolerance(3.0);

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
    double currentDistanceX = drivetrain.getPose().getX();
    double xSpeed = distancePID.calculate(currentDistanceX);

    //TODO: calculating the strafe
    double currentDistanceY = drivetrain.getPose().getY();
    double ySpeed = strafePID.calculate(currentDistanceY);

    //TODO: calculating the angle
    double currentAngle = drivetrain.getPose().getRotation().getDegrees();
    double angleSpeed = anglePID.calculate(currentAngle);

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
    return distancePID.atSetpoint() && strafePID.atSetpoint() && anglePID.atSetpoint();
  }
}
