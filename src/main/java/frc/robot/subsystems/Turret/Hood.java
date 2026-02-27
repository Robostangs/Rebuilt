package frc.robot.subsystems.Turret;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
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
    MotionMagicExpoDutyCycle hoodTargetPosition;

    public Hood() {
        hoodMotor = new TalonFXS(Constants.HoodConstants.hoodMotorID);
        limitSwitch = new DigitalInput(Constants.HoodConstants.kLimitSwitchId);
        hoodTargetPosition = new MotionMagicExpoDutyCycle(hoodMotor.getPosition().getValueAsDouble());
                configureHoodMotor();

    }

    public void configureHoodMotor() {
        TalonFXSConfiguration hoodMotorConfig = new TalonFXSConfiguration();

        hoodMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;

        hoodMotorConfig.Slot0.kP = Constants.HoodConstants.kHoodMotorP;
        hoodMotorConfig.Slot0.kI = Constants.HoodConstants.kHoodMotorI;
        hoodMotorConfig.Slot0.kD = Constants.HoodConstants.kHoodMotorD;
        hoodMotorConfig.Slot0.kS = Constants.HoodConstants.kHoodMotorS;
        hoodMotorConfig.Slot0.kV = Constants.HoodConstants.kHoodMotorV;
        hoodMotorConfig.Slot0.kA = Constants.HoodConstants.kHoodMotorA;

        hoodMotorConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.HoodConstants.kHoodMotionMagicCruiseVelocity;
        hoodMotorConfig.MotionMagic.MotionMagicAcceleration = Constants.HoodConstants.kHoodMotionMagicAcceleration;

        hoodMotorConfig.ExternalFeedback.SensorToMechanismRatio = 44;
        hoodMotor.getConfigurator().apply(hoodMotorConfig);

    }

    public void setHoodStatorCurrentConfig(){
        TalonFXSConfiguration hoodMotorConfig = new TalonFXSConfiguration();
        hoodMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.HoodConstants.kHoodStatorCurrentLimit;

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

    public Supplier<Boolean> CurrentLimitTripped(){
        return () ->hoodMotor.getFault_StatorCurrLimit().getValue();

    }

    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }
    
    public void setMotionMagicTarget(double pos){
        hoodTargetPosition.Position = pos;
    }
    public void setMotionMagic(){
        hoodMotor.setControl(hoodTargetPosition);

    }

    @Override
    public void periodic() {
        // hoodMotor.setControl(hoodTargetPosition);
        SmartDashboard.putNumber("Hood/HoodTargetPosition", hoodTargetPosition.Position);
        SmartDashboard.putNumber("Hood/Hood pos", hoodMotor.getPosition().getValueAsDouble());
    }

}
