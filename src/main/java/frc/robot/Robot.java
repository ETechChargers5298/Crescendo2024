// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.MechConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.LEDStrip.SubsystemPriority;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("MyPose", Pose2d.struct).publish();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    Drivetrain.getInstance().resetIMU();
    Arm.getInstance().setCoast();
    //Arm.getInstance().setValue(MechConstants.START_ANGLE);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    publisher.set(Drivetrain.getInstance().getPose());
    CommandScheduler.getInstance().run();
    
     
    
    // if(Intake.getInstance().checkNoteFound()) {
    //   Launcher.getInstance().launch(MechConstants.LAUNCHER_SPEED);
    // }
  }

  public static boolean isBlue() {
    return DriverStation.getAlliance().isPresent() 
    && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    LEDStrip.request(SubsystemPriority.DEFAULT, LEDStrip.DISABLED);
    Arm.getInstance().setCoast();
  }

  @Override
  public void disabledPeriodic() {
    LEDStrip.request(SubsystemPriority.DEFAULT, LEDStrip.DISABLED);
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.m_autonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    Arm.getInstance().setBrake();
    //  Launcher.getInstance().setBrake();
    //  Launcher.getInstance().toggleRev(true);
    //Drivetrain.getInstance().resetOdometry(new Pose2d());
   // Arm.getInstance().setValue(MechConstants.START_ANGLE);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //  Launcher.getInstance().toggleRev(true);
    //  Launcher.getInstance().toggleBrake(true);
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    Arm.getInstance().setBrake();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    Drivetrain.getInstance().resetOdometry(new Pose2d());
    Drivetrain.getInstance().resetIMU();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
