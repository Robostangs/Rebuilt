package frc.robot.subsystems.Turret;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.TurretCommands.Hood.HomeHood;

@Logged
public class Hood extends SubsystemBase {
    TalonFXS hoodMotor;
    public static Hood mInstance;
    DigitalInput limitSwitch;
    MotionMagicExpoDutyCycle hoodTargetPosition;

     boolean isHomed = false;

    private Hood() {
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

        hoodMotorConfig.CurrentLimits.StatorCurrentLimit = 20;

        hoodMotorConfig.ExternalFeedback.SensorToMechanismRatio = Constants.HoodConstants.kHoodSensortoMechanism;

        hoodMotorConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.HoodConstants.kHoodMotionMagicCruiseVelocity;
        hoodMotorConfig.MotionMagic.MotionMagicAcceleration = Constants.HoodConstants.kHoodMotionMagicAcceleration;

        hoodMotorConfig.ExternalFeedback.SensorToMechanismRatio = 44;
        hoodMotor.getConfigurator().apply(hoodMotorConfig);

    }

    public void setHoodStatorCurrentConfig() {
        TalonFXSConfiguration hoodMotorConfig = new TalonFXSConfiguration();
        hoodMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.HoodConstants.kHoodStatorCurrentLimit;

        hoodMotor.getConfigurator().apply(hoodMotorConfig);

    }

    public double getHoodAngle(double MetersFromHub) {
        double feetFromHub = Units.metersToFeet(MetersFromHub);
        SmartDashboard.putNumber(" feet from hub", feetFromHub);

        return -0.644 + 0.462 * Math.log(feetFromHub);
    }

    public void setHoodDutyCycle(double dc) {
        hoodMotor.set(dc);
    }

    public boolean getIsHomed() {
        return isHomed;
    }
    public static Hood getInstance() {
        if (mInstance == null) {
            mInstance = new Hood();
        }
        return mInstance;
    }

    public void setIsHome(boolean ishome){
        isHomed = ishome;
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Hood/status", status);
    }

    public void setHoodPosition(double pos) {
        hoodMotor.setPosition(pos);
    }

    public double getStatorCurrent() {
        return hoodMotor.getTorqueCurrent().getValueAsDouble();

    }

    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }

    public void setMotionMagicTarget(double pos) {
        hoodTargetPosition.Position = pos;
    }

    public void setMotionMagic() {
        hoodMotor.setControl(hoodTargetPosition);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood/HoodTargetPosition", hoodTargetPosition.Position);
        SmartDashboard.putNumber("Hood/Hood pos", hoodMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Stator Current", getStatorCurrent());
        if(hoodMotor.hasResetOccurred()){
            isHomed = false;
        }

        if(!isHomed){
            CommandScheduler.getInstance().schedule(new HomeHood().withTimeout(4));
        }
 
        
    }

}
