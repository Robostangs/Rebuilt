package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Kirby extends SubsystemBase {
    TalonFX kirbyWheels;
    TalonFXConfiguration kirbyConfiguration;
    TalonFX hopperExtenderMotor;
    TalonFXConfiguration hopperExtenderConfiguration;
    private static Kirby mInstance;
    MotionMagicTorqueCurrentFOC extenderControl;

    /** The intake wheels. */
    public Kirby() {

        kirbyWheels = new TalonFX(Constants.KirbyConstants.kirbyWheelsID);
        kirbyConfiguration = new TalonFXConfiguration();
        kirbyConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.KirbyConstants.KirbyCurrentLimit;
        hopperExtenderMotor = new TalonFX(Constants.KirbyConstants.hopperExtenderID);
        hopperExtenderConfiguration = new TalonFXConfiguration();
        // TODO: Find this john
        hopperExtenderConfiguration.Feedback.SensorToMechanismRatio = Constants.KirbyConstants.extenderSensorToMechanismRatio;
        hopperExtenderConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.KirbyConstants.HopperExtenderCurrentLimit;
        hopperExtenderConfiguration.Slot0.kP = Constants.KirbyConstants.hopperExtenderKP;
        hopperExtenderConfiguration.Slot0.kI = Constants.KirbyConstants.hopperExtenderKI;
        hopperExtenderConfiguration.Slot0.kD = Constants.KirbyConstants.hopperExtenderKD;
        hopperExtenderConfiguration.Slot0.kS = Constants.KirbyConstants.hopperExtenderKS;

        hopperExtenderConfiguration.MotionMagic.MotionMagicCruiseVelocity = Constants.KirbyConstants.kExtenderMotionMagicVelocity;
        hopperExtenderConfiguration.MotionMagic.MotionMagicAcceleration = Constants.KirbyConstants.kExtenderMotionMagicAcceleration;

        hopperExtenderMotor.getConfigurator().apply(hopperExtenderConfiguration);

        extenderControl = new MotionMagicTorqueCurrentFOC(hopperExtenderMotor.getPosition().getValueAsDouble());
    }

    public static Kirby getInstance() {
        if (mInstance == null) {
            mInstance = new Kirby();
        }
        return mInstance;
    }

    public void setKirbyWheels(double speed) {
        kirbyWheels.set(speed);
    }

    public void setHopperExtender(double rots) {
        hopperExtenderMotor.setPosition(rots);
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Kirby/status", status);
    }

    public void setExtenderPosition(double pos) {
        extenderControl.Position = pos;
    }

    public void setIntakeExtenderMode(NeutralModeValue mode) {
        hopperExtenderMotor.setNeutralMode(mode);
    }

    @Override
    public void periodic() {
        hopperExtenderMotor.setControl(extenderControl);

        // johns
    }

}
