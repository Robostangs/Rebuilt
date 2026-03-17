package frc.robot.subsystems.Dumper;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.DumperCommands.Hood.HomeHood;

public class Hood extends SubsystemBase {
    TalonFXS hoodMotorMain;
    TalonFXS hoodMotorSlave;
    public static Hood mInstance;
    MotionMagicExpoDutyCycle hoodTargetPosition;
    static boolean isHomed = false;
    boolean ShouldMM = true;

    private Hood() {
        hoodMotorMain = new TalonFXS(Constants.HoodConstants.hoodMotorMainID);
        hoodMotorSlave = new TalonFXS(Constants.HoodConstants.hoodMotorSlaveID);
        hoodTargetPosition = new MotionMagicExpoDutyCycle(hoodMotorMain.getPosition().getValueAsDouble());
        configureHoodMotors();

    }

    public void configureHoodMotors() {
        TalonFXSConfiguration mainMotorConfig = new TalonFXSConfiguration();

        mainMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;

        mainMotorConfig.Slot0.kP = Constants.HoodConstants.kHoodMotorP;
        mainMotorConfig.Slot0.kI = Constants.HoodConstants.kHoodMotorI;
        mainMotorConfig.Slot0.kD = Constants.HoodConstants.kHoodMotorD;
        mainMotorConfig.Slot0.kS = Constants.HoodConstants.kHoodMotorS;
        mainMotorConfig.Slot0.kV = Constants.HoodConstants.kHoodMotorV;
        mainMotorConfig.Slot0.kA = Constants.HoodConstants.kHoodMotorA;

        mainMotorConfig.CurrentLimits.StatorCurrentLimit = 15;

        mainMotorConfig.ExternalFeedback.SensorToMechanismRatio = Constants.HoodConstants.kHoodSensortoMechanism;

        mainMotorConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.HoodConstants.kHoodMotionMagicCruiseVelocity;
        mainMotorConfig.MotionMagic.MotionMagicAcceleration = Constants.HoodConstants.kHoodMotionMagicAcceleration;

        mainMotorConfig.ExternalFeedback.SensorToMechanismRatio = 44;
        hoodMotorMain.getConfigurator().apply(mainMotorConfig);

        TalonFXSConfiguration slaveMotorConfig = new TalonFXSConfiguration();
        slaveMotorConfig.CurrentLimits.StatorCurrentLimit = 15;


        hoodMotorSlave.getConfigurator().apply(slaveMotorConfig);
        hoodMotorSlave.setControl(new Follower(Constants.HoodConstants.hoodMotorMainID, MotorAlignmentValue.Opposed));



    }

    public void setHoodStatorCurrentConfig() {
        TalonFXSConfiguration hoodMotorConfig = new TalonFXSConfiguration();
        hoodMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.HoodConstants.kHoodStatorCurrentLimit;

        hoodMotorMain.getConfigurator().apply(hoodMotorConfig);

    }

    public double getAngleFromTarget() {
        double feetFromHub = Units.metersToFeet(Robot.getMetersFromTarget());
        SmartDashboard.putNumber("feet from hub", feetFromHub);
        // get regression later
        return 50;
    }

    public void setHoodDutyCycle(double dc) {
        hoodMotorMain.set(dc);
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

    public void setIsHome(boolean ishome) {
        isHomed = ishome;
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Hood/status", status);
    }

    public void setHoodPosition(double pos) {
        hoodMotorMain.setPosition(pos);
    }

    public void setHoodPositionAuto(double pos) {
        hoodMotorMain.setPosition(pos);
        isHomed = true;
    }

    public void setMM() {
        ShouldMM = (!ShouldMM);
    }

    public void setMM(boolean MM) {
        ShouldMM = MM;
    }

    public double getStatorCurrent() {
        return hoodMotorMain.getTorqueCurrent().getValueAsDouble();

    }

    // public boolean getLimitSwitch() {
    // return limitSwitch.get();
    // }

    public void setMotionMagicTarget(double pos) {
        hoodTargetPosition.Position = pos;
    }

    public void setMotionMagic() {

        if (ShouldMM) {

            if (Math.abs(hoodMotorMain.getPosition().getValueAsDouble() - hoodTargetPosition.Position) < 0.04) {
                hoodMotorMain.set(0);
            } else {
                if (hoodTargetPosition.Position > 0.8) {
                    hoodTargetPosition.Position = .8;
                } else if (hoodTargetPosition.Position < 0) {
                    hoodTargetPosition.Position = 0;

                }
                hoodMotorMain.setControl(hoodTargetPosition);

            }
        }

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood/HoodTargetPosition", hoodTargetPosition.Position);
        SmartDashboard.putNumber("Hood/Hood pos", hoodMotorMain.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Stator Current", getStatorCurrent());
        SmartDashboard.putBoolean("Hood/manual", ShouldMM);
        if (hoodMotorMain.hasResetOccurred()) {
            isHomed = false;
        }

        if (!isHomed) {
            CommandScheduler.getInstance().schedule(new HomeHood().withTimeout(4));
        }

    }

}
