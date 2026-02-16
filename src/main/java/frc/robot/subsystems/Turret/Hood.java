package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
    TalonFXS hoodMotor;
    public static Hood mInstance;
    DigitalInput limitSwitch;
    MotionMagicTorqueCurrentFOC hoodTargetPosition;

    public Hood() {
        hoodMotor = new TalonFXS(Constants.HoodConstants.hoodMotorID);
        limitSwitch = new DigitalInput(Constants.HoodConstants.kLimitSwitchId);
        configureHoodMotor();
        hoodTargetPosition = new MotionMagicTorqueCurrentFOC(hoodMotor.getPosition().getValueAsDouble());
    }

    public void configureHoodMotor() {
        TalonFXSConfiguration hoodMotorConfig = new TalonFXSConfiguration();

        hoodMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;

        hoodMotor.getConfigurator().apply(hoodMotorConfig);

        hoodMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;

        hoodMotorConfig.Slot0.kP = Constants.HoodConstants.kHoodMotorP;
        hoodMotorConfig.Slot0.kI = Constants.HoodConstants.kHoodMotorI;
        hoodMotorConfig.Slot0.kD = Constants.HoodConstants.kHoodMotorD;
        hoodMotorConfig.Slot0.kS = Constants.HoodConstants.kHoodMotorS;
        hoodMotorConfig.Slot0.kV = Constants.HoodConstants.kHoodMotorV;
        hoodMotorConfig.Slot0.kA = Constants.HoodConstants.kHoodMotorA;

        hoodMotorConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.HoodConstants.kHoodMotionMagicCruiseVelocity;
        hoodMotorConfig.MotionMagic.MotionMagicAcceleration = Constants.HoodConstants.kHoodMotionMagicAcceleration;
        hoodMotor.getConfigurator().apply(hoodMotorConfig);

    }

    public double getHoodAngle(Pose2d currentPose) {
        // TODO acc fill this out later
        return currentPose.getX();
    }

    public void setHoodDutyCycle(double dc) {
        hoodMotor.set(dc);
    }

    public static Hood getInstance() {
        if (mInstance == null) {
            mInstance = new Hood();
        }
        return mInstance;
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Hood/status", status);
    }

    public void setHoodPosition(double pos) {
        hoodMotor.setPosition(pos);
    }

    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }

    public void periodic() {
        hoodTargetPosition.Position = SmartDashboard.getNumber("Hood setpoint", 0);
        hoodMotor.setControl(hoodTargetPosition);
    }

}
