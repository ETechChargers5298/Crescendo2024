package frc.robot.commands.complex;


import frc.robot.subsystems.Intake;
import frc.robot.Constants.MechConstants;
import frc.robot.commands.basic.IntakeChangeSpeed;
import frc.robot.commands.basic.IntakeEat;
import frc.robot.commands.basic.IntakeNoteStop;
import frc.robot.commands.basic.IntakeSpit;
import frc.robot.commands.basic.LauncherShoot;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class ComplexEat extends SequentialCommandGroup {

    private Intake intake;

    public ComplexEat() {
        intake = Intake.getInstance();
        
            addCommands(

                new IntakeNoteStop(),
                new WaitCommand(0.1),
                //little spit
                new ParallelRaceGroup(
                    new IntakeChangeSpeed(MechConstants.AUTO_INTAKE_SPEED),
                    new IntakeSpit(),
                    new WaitCommand(0.1)
                )

                // //rev the wheels
                // new ParallelRaceGroup(
                //     new LauncherShoot(),
                //     new WaitCommand(1.3)
                // ),

                // //push note into spinning wheels for a launch
                // new ParallelRaceGroup(
                //     new LauncherShoot(),
                //     new IntakeEat(),
                //     new WaitCommand(0.5)
                // )
                
            );
        
    }

}
