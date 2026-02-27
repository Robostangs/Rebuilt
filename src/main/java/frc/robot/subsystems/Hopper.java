package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
    public TalonFX backGoer;
    public TalonFX leftRighter;
    public static Hopper mInstance;

    Alert leftRightHotAlert = new Alert("Hopper LeftRighter Motor is HOT", AlertType.kError);
    Alert backGoerHotAlert = new Alert("Hopper BackGoer Motor is HOT", AlertType.kError);


    public Hopper(){
        backGoer=new TalonFX(Constants.HopperConstants.kBackGoerID);
        leftRighter= new TalonFX(Constants.HopperConstants.kLeftRightID);
        backGoer.getAncillaryDeviceTemp().setUpdateFrequency(50);
        leftRighter.getAncillaryDeviceTemp().setUpdateFrequency(50);
    }
    public void setBackGoerDutyCycle(double dc){
        backGoer.set(dc);
    }

    public void setLeftRighterDutyCycle(double dc){
        leftRighter.set(dc);
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Hopper/status", status);
    }

    public static Hopper getInstance(){
        if (mInstance == null) {
         mInstance = new Hopper();
        }
        return mInstance;
    }
    public void checkIsHot(){
        boolean leftRighterHot=leftRighter.getAncillaryDeviceTemp().getValueAsDouble()>=40;
        boolean backGoerHot=backGoer.getAncillaryDeviceTemp().getValueAsDouble()>=40;

        leftRightHotAlert.set(leftRighterHot);
        backGoerHotAlert.set(backGoerHot);
    }
    @Override
    public void periodic(){
        checkIsHot();
    }
}
