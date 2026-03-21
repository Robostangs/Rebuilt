package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
    public TalonSRX roller;
    public TalonFX rollerL;

    public static Hopper mInstance;

    Alert hopperAlert = new Alert("Hopper roller motor is HOT", AlertType.kError);

    public Hopper() {
        roller = new TalonSRX(1);
        rollerL = new TalonFX(8);

    }

    public void configureLeftRighter() {
        TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();

        rollerConfigs.CurrentLimits.StatorCurrentLimit = 100;
        rollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        // roller.getConfigurator().apply(rollerConfigs);

    }

    
    public void setRollerDutyCycle(double dutyCycle) {
        roller.set(TalonSRXControlMode.PercentOutput, dutyCycle);
        rollerL.set(dutyCycle);
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
