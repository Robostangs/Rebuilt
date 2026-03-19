package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
    public TalonFX roller;
    public static Hopper mInstance;

    Alert hopperAlert = new Alert("Hopper roller motor is HOT", AlertType.kError);

    public Hopper() {
        roller = new TalonFX(Constants.HopperConstants.kRollerID);
    }

    public void configureLeftRighter() {
        TalonFXConfiguration roller = new TalonFXConfiguration();

        roller.CurrentLimits.StatorCurrentLimit = 100;
        roller.CurrentLimits.StatorCurrentLimitEnable = true;

        //TODO bro does not apply configs

    }

    public void setRollerDutyCycle(double dutyCycle) {
        roller.set(dutyCycle);
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
