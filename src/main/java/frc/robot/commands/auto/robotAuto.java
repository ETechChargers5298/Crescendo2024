// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.basic.ArmPivot;
import frc.robot.commands.basic.IntakeEat;
import frc.robot.commands.basic.IntakeSpit;
import frc.robot.commands.basic.LauncherShoot;
import frc.robot.commands.closed.ArmSetAngle;
import frc.robot.commands.closed.ArmSetAngleFromDistance;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class robotAuto extends SequentialCommandGroup {
  /** Creates a new robotauto. */
  public robotAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SequentialCommandGroup(
          new ArmSetAngleFromDistance(),

          new ParallelRaceGroup(
            new LauncherShoot(),
            new WaitCommand(1.0),
            new IntakeSpit(),
            new WaitCommand(0.5)
            ),
          new ArmSetAngle(Constants.MechConstants.FLOOR_ANGLE),

          new ParallelRaceGroup( 
            new IntakeEat()
            )
         

        )
    );
  }
}
