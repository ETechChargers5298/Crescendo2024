package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;


public class Rumble {

    private XboxController controller;
    private boolean isLeftHand;
    private double intensity;


    public Rumble(XboxController controller, boolean isLeftHand, double intensity){
        this.controller = controller;
        this.isLeftHand = isLeftHand;
        this.intensity = intensity;      
    }

    public void rumbleOn(){

        if(isLeftHand){
            controller.setRumble(RumbleType.kLeftRumble, intensity);
        }

        else {
            controller.setRumble(RumbleType.kRightRumble, intensity);
        }
    }

    public void rumbleOff(){
        
            controller.setRumble(RumbleType.kLeftRumble, 0);
            controller.setRumble(RumbleType.kRightRumble, 0);
        
    }

    public void miniRumble(){

        if(isLeftHand){
            controller.setRumble(RumbleType.kLeftRumble, intensity/2);
        }

        else {
            controller.setRumble(RumbleType.kRightRumble, intensity/2);
        }

    }
    

}
