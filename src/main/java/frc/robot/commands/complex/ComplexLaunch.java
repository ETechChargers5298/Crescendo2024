package frc.robot.commands.complex;


import frc.robot.subsystems.Intake;
import frc.robot.commands.basic.IntakeEat;
import frc.robot.commands.basic.IntakeSpit;
import frc.robot.commands.basic.LauncherShoot;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class ComplexLaunch extends SequentialCommandGroup {

    private Intake intake;

    public ComplexLaunch(){
        intake = Intake.getInstance();
        
            addCommands(

                //little spit
                new ParallelRaceGroup(
                    new IntakeSpit(),
                    new WaitCommand(0.1)
                ),

                //rev the wheels
                new ParallelRaceGroup(
                    new LauncherShoot(),
                    new WaitCommand(0.5)
                ),

                //push note into spinning wheels for a launch
                new ParallelRaceGroup(
                    new LauncherShoot(),
                    new IntakeEat(),
                    new WaitCommand(0.5)
                )
                
            );
        
    }

}
