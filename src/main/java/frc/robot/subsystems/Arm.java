package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.subsystems.LEDStrip.SubsystemPriority;
import frc.robot.Constants.MechConstants;
import frc.robot.Constants.VisionConstants;



public class Arm extends SubsystemBase{

        private CANSparkMax leftMotor;
        private CANSparkMax rightMotor;

        private AbsoluteEncoder leftEncoder;
        private AbsoluteEncoder rightEncoder;

        private double angleAverage;
        private static Arm instance;
        private double aprilAngle;

        private Arm(){
          this.leftMotor = new CANSparkMax(Ports.ARM_LEFT, MotorType.kBrushless);
          this.rightMotor = new CANSparkMax(Ports.ARM_RIGHT, MotorType.kBrushless);

          leftMotor.setInverted(true);
          rightMotor.setInverted(false);

          //leftEncoder = leftMotor.getAlternateEncoder(8192);
          rightEncoder = rightMotor.getAbsoluteEncoder(Type.kDutyCycle);
          //leftEncoder = rightEncoder;

          rightEncoder.setPositionConversionFactor(360);
          rightEncoder.setZeroOffset(MechConstants.ARM_OFFSET);

          //leftEncoder.setPositionConversionFactor(360 / (64 / 24));
          

          leftMotor.setIdleMode(IdleMode.kCoast);
          rightMotor.setIdleMode(IdleMode.kCoast);

          // SoftLimitDirection direction = SoftLimitDirection.kReverse;
          // leftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
          // leftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
          // leftMotor.setSoftLimit(SoftLimitDirection.kForward, 106.0f);
          // leftMotor.setSoftLimit(SoftLimitDirection.kReverse, 1.0f);
          
          // rightMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
          // rightMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
          // rightMotor.setSoftLimit(SoftLimitDirection.kForward, 106.0f);
          // rightMotor.setSoftLimit(SoftLimitDirection.kReverse, 1.0f);

          leftMotor.burnFlash();
          rightMotor.burnFlash();

        }

        public static Arm getInstance() {
          if (instance == null) {
            instance = new Arm();
          }
          
          return instance;
          }

          public double getAngle(){
            //angleAverage = (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
            //return angleAverage;
           //temporary change for right encoder
            return rightEncoder.getPosition();
            //leftEncoder.getPosition()
          }

          public void resetValue() {
            
            rightEncoder.setZeroOffset(rightEncoder.getZeroOffset() - rightEncoder.getPosition());
            rightMotor.burnFlash();
          }

          public void setBrake() {
            
            rightMotor.setIdleMode(IdleMode.kBrake);
            leftMotor.setIdleMode(IdleMode.kBrake);
            rightMotor.burnFlash();
            leftMotor.burnFlash();
          }
          public void setCoast() {
            
            rightMotor.setIdleMode(IdleMode.kCoast);
            leftMotor.setIdleMode(IdleMode.kCoast);
            rightMotor.burnFlash();
            leftMotor.burnFlash();
          }

          // public void setValue(double value) {
          //   //leftEncoder.setPosition(0);
          //   rightEncoder.setPosition(value);
          // }

          public void pivot(double speed) {

            leftMotor.set(speed);
            rightMotor.set(speed);
          }
        
          public void stop() {
            leftMotor.set(0);
            rightMotor.set(0);
          }

  public double getArmAprilAngle() {
    aprilAngle = getEquationAngle(Camera.getInstance().getX());
    return aprilAngle;
  }

  public double getEquationAngle(double distance) {
    aprilAngle = VisionConstants.kC + VisionConstants.kB * distance + VisionConstants.kA * Math.pow(distance, 2);
    return aprilAngle;
  }

  public boolean isGoodLaunchAngle(){
    double diff = getAngle() - aprilAngle;
    if (Math.abs(diff) < VisionConstants.LAUNCH_ANGLE_TOLERANCE){
      return true;
    }
    return false;
  }

  public boolean isGoodAmpAngle(){
    double diff = getAngle() - MechConstants.AMP_ANGLE;
    if (Math.abs(diff) < VisionConstants.AMP_ANGLE_TOLERANCE){
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot Value", getAngle());
    SmartDashboard.putNumber("April Arm Angle", getArmAprilAngle());
    SmartDashboard.putBoolean("isGoodAmpAngle", isGoodAmpAngle());
    SmartDashboard.putBoolean("isGoodLaunchAngle", isGoodLaunchAngle());

    
    if (isGoodAmpAngle()) {
      LEDStrip.request(SubsystemPriority.ARM, LEDStrip.AMP_ANGLE);
    } else if (isGoodLaunchAngle()) {
      LEDStrip.request(SubsystemPriority.ARM, LEDStrip.SPEAKER_ANGLE);
    }
    

  }   
}
