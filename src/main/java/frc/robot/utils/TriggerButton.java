// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class TriggerButton extends Trigger {

    private XboxController controller;
    private int axis;
    public static final String Left = "LEFT";
    public static final String Right = "RIGHT";
    
    /**
     * Turns trigger into button
     * @author Cat Ears and Tahlei
     * @param controller
     * @param axis (2 = LTrig, 3 = RTrig)
     */
    public TriggerButton(XboxController controller, int axis){
      super(() -> controller.getRawAxis(axis) > 0.7);
        this.controller = controller;
        this.axis = axis;
        //this.isLeft = isLeft;
    }

    // @Override
    // public boolean get(){
    //     // After press 0.5 on trigger value is true
    //    if(side.equals(Left)){
    //     return controller.getLeftTriggerAxis() >= 0.5;
    //    }
    //    else if (side.equals(Right)){
    //     return controller.getRightTriggerAxis() >= 0.5;
    //    }
    //    return false;
    // }
}