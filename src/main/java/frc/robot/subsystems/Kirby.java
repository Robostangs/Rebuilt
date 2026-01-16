package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Kirby extends SubsystemBase {
    TalonFX kirbyWheels;
    TalonFXConfiguration kirbyConfiguration;
    private static Kirby mInstance;

    /** The intake wheels. */
    public Kirby() {
        kirbyWheels = new TalonFX(Constants.KirbyConstants.kirbyWheelsID);
        kirbyConfiguration = new TalonFXConfiguration();
        kirbyConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.KirbyConstants.KirbyCurrentLimit;
    }

    public void setKirbyWheels(double speed) {
        kirbyWheels.set(speed);
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Kirby/status", status);
    }

    public static Kirby getInstance() {
        if (mInstance == null) {
            mInstance = new Kirby();
        }
        return mInstance;
    }

}
