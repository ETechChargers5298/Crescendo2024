// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.MechConstants;
import frc.robot.commands.basic.IntakeEat;
import frc.robot.commands.basic.IntakeSpit;
import frc.robot.commands.basic.LauncherShoot;
import frc.robot.commands.closed.ArmSetAngle;
import frc.robot.commands.closed.ArmSetAngleApril;
import frc.robot.commands.closed.DrivePID;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class Auto2NoteMid extends SequentialCommandGroup {
  /** Creates a new robotauto. */
  public Auto2NoteMid() {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SequentialCommandGroup(

          //move arm to correct angle to shoot
          new ParallelRaceGroup(
            new ArmSetAngle(MechConstants.LAUNCH_ANGLE),
            new WaitCommand(1.5)
            ),
          

          //Rev flywheel
          new ParallelRaceGroup(
            new LauncherShoot(),
            new WaitCommand(1.0)
          ),

          //Launch Note
          new ParallelRaceGroup(
            new IntakeEat(),
            new WaitCommand(0.5)
          ),

          //Put arm down to ground
          new ParallelRaceGroup(
          new ArmSetAngle(Constants.MechConstants.FLOOR_ANGLE),
          new WaitCommand(1.5)
          ),
          
          new WaitCommand(1.2),
          
          //Start to eat & Drive forward           
          new ParallelRaceGroup(
            new IntakeEat(),
            new DrivePID(2.0, 0.0, 0.0),
            new WaitCommand(1.0)
          ),

          //Move Arm Angle Up to Launch
          new ArmSetAngleApril(),

          //Launch 2nd Note          
          new ParallelRaceGroup(
            new LauncherShoot(),
            new WaitCommand(1.5)
          ),
          new ParallelRaceGroup(
            new IntakeEat(),
            new WaitCommand(0.5)
          )
         

        )
    );
  }
}
