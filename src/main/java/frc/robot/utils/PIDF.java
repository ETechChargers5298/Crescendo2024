// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkPIDController;

/** Add your docs here. */
public class PIDF { 

    public final double kP;
    public final double kI;
    public final double kD;
    public final double kF;
    
    public final double inputRange[];

    public final double outputRange[];

    public final boolean wrappingEnabled;

    public PIDF (double kP) {

        this.kP = kP;

        kI = 0.0;
        kD = 0.0;
        kF = 0.0;

        inputRange = new double[2];
        inputRange[0] = Double.NaN;
        inputRange[1] = Double.NaN;

        outputRange = new double[2];
        outputRange[0] = -1;
        outputRange[1] = 1;

        wrappingEnabled = false;
    }

    public PIDF (double kP, double kI, double kD) {

        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        kF = 0.0;
        
        inputRange = new double[2];
        inputRange[0] = Double.NaN;
        inputRange[1] = Double.NaN;

        outputRange = new double[2];
        outputRange[0] = -1;
        outputRange[1] = 1;

        wrappingEnabled = false;
        
    }

    public PIDF (double kP, double kI, double kD, double kF) {

        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        
        inputRange = new double[2];
        inputRange[0] = Double.NaN;
        inputRange[1] = Double.NaN;

        outputRange = new double[2];
        outputRange[0] = -1;
        outputRange[1] = 1;

        wrappingEnabled = false;
        
    }

    public PIDF (double kP, double kI, double kD, double kF, double minO, double maxO) {

        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;

        inputRange = new double[2];
        inputRange[0] = Double.NaN;
        inputRange[1] = Double.NaN;

        outputRange = new double[2];
        outputRange[0] = minO;
        outputRange[1] = maxO;

        wrappingEnabled = false;
    }

    public PIDF (double kP, double minIn, double maxIn, double minO, double maxO, boolean wrappingEnabled) {
        this.kP = kP;
        kI = 0.0;
        kD = 0.0;
        kF = 0.0;
        
        inputRange = new double[2];
        inputRange[0] = minIn;
        inputRange[1] = maxIn;

        outputRange = new double[2];
        outputRange[0] = minO;
        outputRange[1] = maxO;

        this.wrappingEnabled = wrappingEnabled;
    }

    public PIDF (double kP, double kI, double kD, double kF, double minIn, double maxIn, double minO, double maxO, boolean wrappingEnabled) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        
        inputRange = new double[2];
        inputRange[0] = minIn;
        inputRange[1] = maxIn;

        outputRange = new double[2];
        outputRange[0] = minO;
        outputRange[1] = maxO;

        this.wrappingEnabled = wrappingEnabled;
    }

    public SparkPIDController getConfiguredController(CANSparkMax motor, MotorFeedbackSensor sensor) {
        SparkPIDController pidController = motor.getPIDController();

        pidController.setFeedbackDevice(sensor);
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setFF(kF);

        if(wrappingEnabled) {
            pidController.setPositionPIDWrappingMinInput(inputRange[0]);
            pidController.setPositionPIDWrappingMaxInput(inputRange[1]);
            pidController.setPositionPIDWrappingEnabled(true);
        }

        pidController.setOutputRange(outputRange[0], outputRange[1]);

        return pidController;
    }


}