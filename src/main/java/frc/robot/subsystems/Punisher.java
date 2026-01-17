package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The turret
 */
public class Punisher extends SubsystemBase {

    /**
     * Shooter motor
     */
    TalonFX punisherMotor;
    /**
     * Feeder motor
     */
    TalonFX jigsaswMotor;
    // TalonFX microMotor;

    TalonFXConfiguration jigsawConfig;

    private static Punisher mInstance;

    /**
     * The turret
     */
    public Punisher() {
        punisherMotor = new TalonFX(Constants.PunisherConstants.punisherMotorID);
        jigsaswMotor = new TalonFX(Constants.PunisherConstants.jigsaswMotorID);

        jigsawConfig = new TalonFXConfiguration();

        // jigsawConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        jigsaswMotor.getConfigurator().apply(jigsawConfig);

    }

    public static Punisher getInstance() {
        if (mInstance == null) {
            mInstance = new Punisher();
        }
        return mInstance;
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Punisher/status", status);
    }

    public void setPunisherdutycycle(double speed) {
        punisherMotor.set(speed);
    }

    public void setJigsawdutycycle(double speed) {
        jigsaswMotor.set(speed);

    }

}
