// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closed;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;

public class MoveToAmp extends Command {

  private Drivetrain drivetrain;
  private double desiredDistanceMeters; 
  private double desiredStrafeMeters;
  //private double desiredAngleDegrees;
  private PIDController distancePID, strafePID, anglePID;
  private double distanceSetPoint, strafeSetPoint;
  private double startDistance;
  private double startStrafe;
  private double startAngle;
  private Camera cam;


  /** Creates a new DrivePID. */
  public MoveToAmp() {
    
    //Instantiates the drivetrain object
    drivetrain = Drivetrain.getInstance();
    cam = Camera.getInstance();

    //saves desired pose values
    this.desiredDistanceMeters = desiredDistanceMeters;
    this.desiredStrafeMeters = desiredStrafeMeters;
    //this.desiredAngleDegrees = desiredAngleDegrees;

    //setup for Distance PID Controller
    this.distancePID = new PIDController(1.0, 0.0, 0.0);
    startDistance = drivetrain.getPose().getX();

    //sets the allignment for forward-back of chassis to get to middle of amp
    if(cam.getAllianceColor().equals("BLUE")) {
      distanceSetPoint = startDistance + cam.getDesiredY(cam.getDesiredTarget(6));
     } else if(cam.getAllianceColor().equals("RED")) {
       distanceSetPoint = startDistance + cam.getDesiredY(cam.getDesiredTarget(5));
     }

    this.distancePID.setSetpoint(distanceSetPoint);
    this.distancePID.setTolerance(0.05);

    //TODO: setup for Strafe PID Controller
    this.strafePID = new PIDController(0.5, 0.0, 0.0);
    startStrafe = drivetrain.getPose().getY();

    // sets the left-right motion of chassis to move towards amp
    if(cam.getAllianceColor().equals("BLUE")) {
      strafeSetPoint = startDistance + cam.getDesiredX(cam.getDesiredTarget(6));
     } else if(cam.getAllianceColor().equals("RED")) {
       strafeSetPoint = startDistance + cam.getDesiredX(cam.getDesiredTarget(5));
     }

    this.strafePID.setSetpoint(strafeSetPoint);
    this.strafePID.setTolerance(0.1);

    //TODO: setup for Angle PID Controller
    this.anglePID = new PIDController(0.005, 0.0, 0.0);
    startAngle = drivetrain.getPose().getRotation().getDegrees();

    // setting the angle setpoint to be 90 or 270 based on the alliance(facing the amp)
    if(cam.getAllianceColor().equals("BLUE")) {
      this.anglePID.setSetpoint(270);
     } else if(cam.getAllianceColor().equals("RED")) {
      this.anglePID.setSetpoint(90);
     }
    
    this.anglePID.setTolerance(3.0);

    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.stopDrive();
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
    drivetrain.setXSpeed(xSpeed);
    drivetrain.setYSpeed(ySpeed);
    drivetrain.setRotSpeed(angleSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    return distancePID.atSetpoint() && strafePID.atSetpoint() && anglePID.atSetpoint();
  }
}
