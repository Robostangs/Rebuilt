package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
    public VictorSPX backGoer;
    public VictorSPX leftRighter;
    public static Hopper mInstance;

    Alert leftRightHotAlert = new Alert("Hopper LeftRighter Motor is HOT", AlertType.kError);
    Alert backGoerHotAlert = new Alert("Hopper BackGoer Motor is HOT", AlertType.kError);


    public Hopper(){
        backGoer = new VictorSPX(Constants.HopperConstants.kBackGoerID);
        leftRighter= new VictorSPX(Constants.HopperConstants.kLeftRightID);    
    }
    public void setBackGoerDutyCycle(double dc){
        backGoer.set(ControlMode.PercentOutput, dc);
    }

    public void setLeftRighterDutyCycle(double dc){
        leftRighter.set(ControlMode.PercentOutput, dc);
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
    public void checkIsHot(){
        boolean leftRighterHot=leftRighter.getTemperature()>=40;
        boolean backGoerHot=backGoer.getTemperature()>=40;

        leftRightHotAlert.set(leftRighterHot);
        backGoerHotAlert.set(backGoerHot);
    }
    @Override
    public void periodic(){
        checkIsHot();
    }
}
