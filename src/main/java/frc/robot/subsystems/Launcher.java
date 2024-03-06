package frc.robot.subsystems;

import com.fasterxml.jackson.databind.AnnotationIntrospector.ReferenceProperty.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Launcher extends SubsystemBase {

    private static Launcher instance;
    private CANSparkMax topMotor;
    private CANSparkMax bottomMotor;
    private RelativeEncoder topEncoder;
    private RelativeEncoder bottomEncoder;

    private Launcher(){

        topMotor = new CANSparkMax(Ports.TOP_LAUNCHER_MOTOR_PORT, MotorType.kBrushless);
        bottomMotor = new CANSparkMax(Ports.BOTTOM_LAUNCHER_MOTOR_PORT, MotorType.kBrushless);

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

    public void launch(double speed){
        topMotor.set(speed);
        bottomMotor.set(speed);

    }

    public void setBrake() {
        topMotor.setIdleMode(IdleMode.kBrake);
        bottomMotor.setIdleMode(IdleMode.kBrake);

        topMotor.burnFlash();
        bottomMotor.burnFlash();
    }

    public void setCoast() {
        topMotor.setIdleMode(IdleMode.kCoast);
        bottomMotor.setIdleMode(IdleMode.kCoast);

        topMotor.burnFlash();
        bottomMotor.burnFlash();
    }

    public double getBottomEncoder(){
        double bottomPosition = bottomEncoder.getPosition();
        return bottomPosition;
    }

    public double getTopEncoder(){
        double topPosition = topEncoder.getPosition();
        return topPosition;
    }

    public double getBottomSpeed(){
        return bottomEncoder.getVelocity();
    }

    public double getTopSpeed(){
        return topEncoder.getVelocity();
    }

    public void stop(){
        topMotor.set(0);
        bottomMotor.set(0);
    }

    
    @Override
    public void periodic() {
        //code to update SmartDashboard
        SmartDashboard.putNumber("LaunchTopSpeed", getTopSpeed());
        SmartDashboard.putNumber("LaunchBotSpeed", getBottomSpeed());
        SmartDashboard.putString("top idle mode", topMotor.getIdleMode().toString());
        SmartDashboard.putString("bot idle mode", bottomMotor.getIdleMode().toString());

    }

}