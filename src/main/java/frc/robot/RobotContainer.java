// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArmJoystick;
// import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmPivot;
import frc.robot.commands.ArmPivotReverse;
import frc.robot.commands.ArmSetAngle;
// import frc.robot.commands.Autos;
// import frc.robot.commands.ExampleCommand;
// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeEat;
import frc.robot.commands.IntakeSpit;
import frc.robot.commands.LauncherShoot;
import frc.robot.commands.LauncherTake;
import frc.robot.Ports;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.MechConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.MoveToTarget;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.DPad;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Camera;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}d                            
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //Auto Command Fields
  private final Command moveToTarget;
  //private final Camera Careywashere = Camera.getInstance();
  
  //XBox Controller Fields
  private static final XboxController driverController = new XboxController(Ports.DRIVER_CONTROLLER);
  private static final XboxController operatorController = new XboxController(Ports.OPERATOR_CONTROLLER);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
public RobotContainer () { 

    //Instantiate Auto Commands here
    moveToTarget = new MoveToTarget();

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

    //TODO
    //lock & unlock wheels with X/Y

    //TODO
    //auto-align drivetrain to speaker target/greenzone with A


    //intake eat/spit with LB/RB
    new JoystickButton(operatorController,Button.kLeftBumper.value).whileTrue(new IntakeSpit());
    new JoystickButton(operatorController,Button.kRightBumper.value).whileTrue(new IntakeEat());


    //----- OPERATOR CONTROLS -----//

    //intake eat/spit with B/X
    new JoystickButton(operatorController,Button.kB.value).whileTrue(new IntakeEat());
    new JoystickButton(operatorController,Button.kX.value).whileTrue(new IntakeSpit());

    //launcher shoot/take with A/Y
    new JoystickButton(operatorController,Button.kY.value).whileTrue(new LauncherShoot());
    new JoystickButton(operatorController,Button.kA.value).whileTrue(new LauncherTake());

    //TODO
    //auto launch sequence with RB

    //arm Pivot with LB/RB
    new JoystickButton(operatorController,Button.kLeftBumper.value).whileTrue(new ArmPivotReverse());
    new JoystickButton(operatorController,Button.kRightBumper.value).whileTrue(new ArmPivot());

    //arm autoPivot with DPAD
    new DPad(operatorController, 0).whileTrue(new ArmSetAngle(MechConstants.START_ANGLE));
    new DPad(operatorController, 90).whileTrue(new ArmSetAngle(MechConstants.AMP_ANGLE));
    new DPad(operatorController, 180).whileTrue(new ArmSetAngle(MechConstants.FLOOR_ANGLE));
    new DPad(operatorController, 270).whileTrue(new ArmSetAngle(MechConstants.LAUNCH_ANGLE));

    //arm pivot up/down with joystick (LX)
    Arm.getInstance().setDefaultCommand(new ArmJoystick( () -> operatorController.getLeftX()));

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

  public Command getAutonomousCommand() {
    
    //TODO
    //Setup "Sendable Chooser" for different commands to be run based on SmartDashboard
    
    return moveToTarget;



    /* //FROM REV EXAMPLE 
     *
    */
    // Create config for trajectory
//     TrajectoryConfig config = new TrajectoryConfig(
//         AutoConstants.kMaxSpeedMetersPerSecond,
//         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//         // Add kinematics to ensure max speed is actually obeyed
//         .setKinematics(SwerveConstants.DRIVE_KINEMATICS);

//     // An example trajectory to follow. All units in meters.
//     Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
//         // Start at the origin facing the +X direction
//         new Pose2d(0, 0, new Rotation2d(0)),
//         // Pass through these two interior waypoints, making an 's' curve path
//         List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
//         // End 3 meters straight ahead of where we started, facing forward
//         new Pose2d(3, 0, new Rotation2d(0)),
//         config);

//     var thetaController = new ProfiledPIDController(
//         AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
//     thetaController.enableContinuousInput(-Math.PI, Math.PI);

//     SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
//         exampleTrajectory,
//         Drivetrain.getInstance()::getPose, // Functional interface to feed supplier
//         SwerveConstants.DRIVE_KINEMATICS,

//         // Position controllers
//         new PIDController(AutoConstants.kPXController, 0, 0),
//         new PIDController(AutoConstants.kPYController, 0, 0),
//         thetaController,
//         Drivetrain.getInstance()::setModuleStates,
//         Drivetrain.getInstance());

//     // Reset odometry to the starting pose of the trajectory.
//     Drivetrain.getInstance().resetOdometry(exampleTrajectory.getInitialPose());

//     // Run path following command, then stop at the end.
//     return swerveControllerCommand.andThen(() -> Drivetrain.getInstance().drive(0, 0, 0, false, false));

  //  */

  }
}
