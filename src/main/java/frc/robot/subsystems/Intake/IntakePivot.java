package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakePivot extends SubsystemBase {
    TalonFX hopperExtenderMotor;
    TalonFXConfiguration hopperExtenderConfig;
    private static IntakePivot mInstance;
    MotionMagicTorqueCurrentFOC extenderControl;

    Alert hopperExtenderAlert = new Alert("Hopper Extender Motor is HOT", AlertType.kError);

    public IntakePivot() {
        extenderControl = new MotionMagicTorqueCurrentFOC(hopperExtenderMotor.getPosition().getValueAsDouble());
        setIntakePivotConfig();

    }

    public static IntakePivot getInstance() {
        if (mInstance == null) {
            mInstance = new IntakePivot();
        }
        return mInstance;
    }

    public void setIntakePivotConfig() {
        hopperExtenderMotor = new TalonFX(Constants.KirbyConstants.hopperExtenderID);
        hopperExtenderConfig = new TalonFXConfiguration();

        // TODO: Find this john
        hopperExtenderConfig.Feedback.SensorToMechanismRatio = Constants.KirbyConstants.extenderSensorToMechanismRatio;
        hopperExtenderConfig.CurrentLimits.SupplyCurrentLimit = Constants.KirbyConstants.HopperExtenderCurrentLimit;
        hopperExtenderConfig.Slot0.kP = Constants.KirbyConstants.hopperExtenderKP;
        hopperExtenderConfig.Slot0.kI = Constants.KirbyConstants.hopperExtenderKI;
        hopperExtenderConfig.Slot0.kD = Constants.KirbyConstants.hopperExtenderKD;
        hopperExtenderConfig.Slot0.kS = Constants.KirbyConstants.hopperExtenderKS;
        hopperExtenderConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.KirbyConstants.kExtenderMotionMagicVelocity;
        hopperExtenderConfig.MotionMagic.MotionMagicAcceleration = Constants.KirbyConstants.kExtenderMotionMagicAcceleration;

        hopperExtenderMotor.getConfigurator().apply(hopperExtenderConfig);
    }

    public void setDC(double dutyCycle) {
        hopperExtenderMotor.set(dutyCycle);
    }

    public void setHopperExtender(double rots) {
        hopperExtenderMotor.setPosition(rots);
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Intake Pivot/Status", status);
    }

    public void setExtenderPosition(double pos) {
        extenderControl.Position = pos;
    }

    public void setIntakeExtenderMode(NeutralModeValue mode) {
        hopperExtenderMotor.setNeutralMode(mode);
    }

    public void checkIfMotorHot() {
        boolean isMotorHot = hopperExtenderMotor.getAncillaryDeviceTemp().getValueAsDouble() >= 45;
        hopperExtenderAlert.set(isMotorHot);
        SmartDashboard.putBoolean("Intake Pivot/isMotorHot", isMotorHot);
    }

    @Override
    public void periodic() {
        hopperExtenderMotor.setControl(extenderControl);
        checkIfMotorHot();
    }
}
