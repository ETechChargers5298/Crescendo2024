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
import frc.robot.commands.basic.LauncherShoot;
import frc.robot.commands.closed.ArmSetAngle;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class AutoFirstShot extends SequentialCommandGroup {
  /** Creates a new robotauto. */
  public AutoFirstShot() {

    addCommands(
        new SequentialCommandGroup(

          //move arm to correct angle to shoot
          new ParallelRaceGroup(
            new ArmSetAngle(MechConstants.LAUNCH_ANGLE + 2),
            new LauncherShoot(),
            new WaitCommand(2.5)
          ),

          //Launch Note
          new ParallelRaceGroup(
            new IntakeEat(),
            new WaitCommand(0.7)
          ),

          //Put arm down to ground
          new ParallelRaceGroup(
          new ArmSetAngle(Constants.MechConstants.FLOOR_ANGLE),
          new WaitCommand(1.5)
          )

        )
    );
  }
}
