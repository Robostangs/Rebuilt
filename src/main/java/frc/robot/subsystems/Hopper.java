package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
    public TalonSRX backGoer;
    public TalonFX leftRighter;
    public static Hopper mInstance;

    Alert leftRightHotAlert = new Alert("Hopper LeftRighter Motor is HOT", AlertType.kError);
    Alert backGoerHotAlert = new Alert("Hopper BackGoer Motor is HOT", AlertType.kError);


    public Hopper(){
        backGoer = new TalonSRX(Constants.HopperConstants.kBackGoerID);
        leftRighter= new TalonFX(Constants.HopperConstants.kLeftRightID);    
    }

 
    public void setLeftRightDutyCycle(double dutyCycle){
        leftRighter.set(dutyCycle);
    }

    public void setBackGoerDutyCycle(double dutyCycle){
        backGoer.set(ControlMode.PercentOutput, dutyCycle);
    }

    public void postLeftRighterStatus(String status) {
        SmartDashboard.putString("Hopper/leftRighterStatus", status);
    }
    public void postBackGoerStatus(String status) {
        SmartDashboard.putString("Hopper/backGoerStatus", status);
    }

    public static Hopper getInstance(){
        if (mInstance == null) {
         mInstance = new Hopper();
        }
        return mInstance;
    }
    // public void checkIsHot(){
    //     boolean leftRighterHot=leftRighter.getTemperature()>=40;
    //     boolean backGoerHot=backGoer.getTemperature()>=40;

    //     leftRightHotAlert.set(leftRighterHot);
    //     backGoerHotAlert.set(backGoerHot);
    // }
    @Override
    public void periodic(){
        // checkIsHot();
    }
}
