// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.basic.*;
import frc.robot.commands.closed.*;
import frc.robot.commands.closed.TurnToAngle.DriveAngle;
import frc.robot.commands.complex.*;
import frc.robot.commands.auto.*;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.utils.*;

import java.util.List;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}d                            
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private static final XboxController driverController = new XboxController(Ports.DRIVER_CONTROLLER);
  private static final XboxController operatorController = new XboxController(Ports.OPERATOR_CONTROLLER);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    //See button mappings on team Google Drive file:
    // https://docs.google.com/presentation/d/1RsC4LT027S3UziIJyp8OQ5cZ9MOVuiTcsnDxlTtHj_I/edit#slide=id.g18d2b75b637cb431_3

    //----- DRIVER CONTROLS -----//
    //Drive robot with forward (LY), strafe (LX), and turn (RX)
    Drivetrain.getInstance().setDefaultCommand(new SwerveDrive(
      () -> -driverController.getRawAxis(1),
      () -> -driverController.getRawAxis(0),
      () -> -driverController.getRawAxis(4)
    ));

    //Snap robot to specific angle
    new DPad(driverController, 0).whileTrue(new TurnToAngle(DriveAngle.FRONT));
    new DPad(driverController, 90).whileTrue(new TurnToAngle(DriveAngle.LEFT));
    new DPad(driverController, 180).whileTrue(new TurnToAngle(DriveAngle.BACK));
    new DPad(driverController, 270).whileTrue(new TurnToAngle(DriveAngle.RIGHT));
    
    //Driver control of intake of notes with LB/RB
    new JoystickButton(driverController,Button.kLeftBumper.value).whileTrue(new IntakeSpit());
    new JoystickButton(driverController,Button.kRightBumper.value).whileTrue(new IntakeEat());
    
    //TODO
    //lock & unlock wheels with X/Y

    //auto-align drivetrain to speaker target/greenzone with A
    new JoystickButton(driverController,Button.kA.value).whileTrue(new MoveToTarget());



    //----- OPERATOR CONTROLS -----//

    //intake eat/spit with B/X
    new JoystickButton(operatorController,Button.kB.value).whileTrue(new IntakeEat());
    new JoystickButton(operatorController,Button.kX.value).whileTrue(new IntakeSpit());

    //launcher shoot/take with A/Y
    new JoystickButton(operatorController,Button.kY.value).whileTrue(new LauncherShoot());
    new JoystickButton(operatorController,Button.kA.value).whileTrue(new LauncherTake());

    //auto launch sequence with RB
    new JoystickButton(operatorController,Button.kRightBumper.value).whileTrue(new ComplexLaunch());

    // arm Pivot with LB/RB
    // new JoystickButton(operatorController,Button.kLeftBumper.value).whileTrue(new ArmPivotReverse());
    // new JoystickButton(operatorController,Button.kRightBumper.value).whileTrue(new ArmPivot());

    //arm autoPivot with DPAD
    new DPad(operatorController, 0).whileTrue(new ArmSetAngle(MechConstants.START_ANGLE));
    new DPad(operatorController, 90).whileTrue(new ArmSetAngle(MechConstants.AMP_ANGLE));
    new DPad(operatorController, 180).whileTrue(new ArmSetAngle(MechConstants.FLOOR_ANGLE));
    new DPad(operatorController, 270).whileTrue(new ArmSetAngle(MechConstants.LAUNCH_ANGLE));

    //arm pivot up/down with joystick (LY)
    Arm.getInstance().setDefaultCommand(new ArmJoystick( () -> operatorController.getLeftY()));

    //TODO
    //auto arm pivot based on apriltags with LB

    //TODO
    //climber up & down with joystick (RY)
  

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */


  public Command m_autonomousCommand() {
    // An example command will be run in autonomous

    return new MoveToTarget();


  }
}
