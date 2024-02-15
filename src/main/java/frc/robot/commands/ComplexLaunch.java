package frc.robot.commands;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.MechConstants;
import frc.robot.subsystems.Launcher;
import frc.robot.Ports;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;



public class ComplexLaunch extends SequentialCommandGroup {

    public ComplexLaunch(){
        if(Intake.isNoteFound){
        addCommands(
            new LauncherShoot(),
            new WaitCommand(0.5),
            new IntakeEat()
        );
        }

    }

  
 
    

}
