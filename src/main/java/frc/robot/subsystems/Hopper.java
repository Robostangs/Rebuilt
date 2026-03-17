package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
    public TalonFX roller1;
    public TalonFX roller2;
    public static Hopper mInstance;

    Alert hopper1Alert = new Alert("Hopper roller motor 1is HOT", AlertType.kError);
    Alert hopper2Alert = new Alert("Hopper roller motor 2 is HOT", AlertType.kError);


    public Hopper() {
        roller1 = new TalonFX(Constants.HopperConstants.kRollerID);
        roller2 = new TalonFX(Constants.HopperConstants.kRollerID + 1);
    }

    public void configureLeftRighter() {
        TalonFXConfiguration roller1Config = new TalonFXConfiguration();
        TalonFXConfiguration roller2Config = new TalonFXConfiguration();

        //TODO figure out
        // roller1Config.CurrentLimits.StatorCurrentLimit = 100;
        // roller1Config.CurrentLimits.StatorCurrentLimitEnable = true;

        // roller2Config.CurrentLimits.StatorCurrentLimit = 100;
        // roller2Config.CurrentLimits.StatorCurrentLimitEnable = true;
    }

    public void setRollerDutyCycle(double dutyCycle) {
        roller1.set(dutyCycle);
        roller2.set(dutyCycle);
    }

    public void postRollerStatus(String status) {
        SmartDashboard.putString("Hopper/rollerStatus", status);
    }

    public static Hopper getInstance() {
        if (mInstance == null) {
            mInstance = new Hopper();
        }
        return mInstance;
    }
    @Override
    public void periodic() {
    }
}
