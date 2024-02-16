package frc.robot.commands.closed;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.MechConstants;
import frc.robot.commands.basic.IntakeEat;
import frc.robot.commands.basic.LauncherShoot;
import frc.robot.subsystems.Launcher;
import frc.robot.Ports;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;



public class ComplexLaunch extends SequentialCommandGroup {

    public ComplexLaunch(){
        if(Intake.isNoteFound){
        addCommands(
            //put these together in a sequential race command
            new LauncherShoot(),
            new WaitCommand(0.5),

            //then do thise command -- with a sequential race
            new IntakeEat()
        );
        }

    }

  
 
    

}
