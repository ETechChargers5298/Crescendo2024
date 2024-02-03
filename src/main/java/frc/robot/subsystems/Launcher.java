package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Launcher extends SubsystemBase {
    private static Launcher instance;
private CANSparkMax topMotor;
private CANSparkMax bottomMotor;
private RelativeEncoder topEncoder;
private RelativeEncoder bottomEncoder;

private Launcher(){

topMotor = new CANSparkMax(Constants.MechConstants.TOP_LAUNCHER_MOTOR_PORT, MotorType.kBrushless);
bottomMotor = new CANSparkMax(Constants.MechConstants.BOTTOM_LAUNCHER_MOTOR_PORT, MotorType.kBrushless);

topEncoder = topMotor.getEncoder();
bottomEncoder = bottomMotor.getEncoder();
}

public static Launcher getInstance(){
    if(instance == null) {
        instance = new Launcher();
    }
    return instance;
}

public void take(double speed){
    topMotor.set(-speed);
    bottomMotor.set(-speed);
}


public void shoot(double speed){
    topMotor.set(speed);
    bottomMotor.set(speed);

}

public double getBottomEncoder(){
double bottomPosition = bottomEncoder.getPosition();
return bottomPosition;
}

public double getTopEncoder(){
double topPosition = topEncoder.getPosition();
return topPosition;

}



public void stop(){
    topMotor.set(0);
    bottomMotor.set(0);
}


}