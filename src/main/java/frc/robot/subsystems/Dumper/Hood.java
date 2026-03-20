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
    TalonFXS mainMotor;
    TalonFXS slaveMotor;
    public static Hood mInstance;
    MotionMagicExpoDutyCycle hoodTargetPosition;

    static boolean isHomed = false;
    Follower slaveControl;
    boolean ShouldMM = true;

    private Hood() {
        mainMotor = new TalonFXS(Constants.HoodConstants.mainMotorID);
        slaveMotor = new TalonFXS(Constants.HoodConstants.hoodSlaveMotorID);
        hoodTargetPosition = new MotionMagicExpoDutyCycle(mainMotor.getPosition().getValueAsDouble());
        configureHoodMotor();
        slaveControl = new Follower(Constants.HoodConstants.mainMotorID, MotorAlignmentValue.Opposed);
    }

    public void configureHoodMotor() {
        TalonFXSConfiguration mainMotorConfig = new TalonFXSConfiguration();

        mainMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;

        mainMotorConfig.Slot0.kP = Constants.HoodConstants.kHoodMotorP;
        mainMotorConfig.Slot0.kI = Constants.HoodConstants.kHoodMotorI;
        mainMotorConfig.Slot0.kD = Constants.HoodConstants.kHoodMotorD;
        mainMotorConfig.Slot0.kS = Constants.HoodConstants.kHoodMotorS;
        mainMotorConfig.Slot0.kV = Constants.HoodConstants.kHoodMotorV;
        mainMotorConfig.Slot0.kA = Constants.HoodConstants.kHoodMotorA;

        // TODO make this constant
        mainMotorConfig.CurrentLimits.StatorCurrentLimit = 15;

        mainMotorConfig.ExternalFeedback.SensorToMechanismRatio = Constants.HoodConstants.kHoodSensortoMechanism;

        mainMotorConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.HoodConstants.kHoodMotionMagicCruiseVelocity;
        mainMotorConfig.MotionMagic.MotionMagicAcceleration = Constants.HoodConstants.kHoodMotionMagicAcceleration;

        mainMotorConfig.ExternalFeedback.SensorToMechanismRatio = 44;
        mainMotor.getConfigurator().apply(mainMotorConfig);

        TalonFXSConfiguration slaveMotorConfig = new TalonFXSConfiguration();
        slaveMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        slaveMotorConfig.CurrentLimits.StatorCurrentLimit = 15;

        slaveMotor.getConfigurator().apply(slaveMotorConfig);
        slaveMotor.setControl(new Follower(Constants.HoodConstants.mainMotorID, MotorAlignmentValue.Opposed));
    }

    public double getHoodAngle(double metersFromTarget) {
        double feetFromTarget = Units.metersToFeet(metersFromTarget);
        if (Robot.isPassing() && feetFromTarget > Constants.FieldConstants.maxRegressionDistance) {
            // full send if we on opposing alliance and we're past the max actual distance
            // for passing
            return Constants.HoodConstants.kPosAtOppositeAlliancePass;
        } else {

            return -0.522 + 0.429 * Math.log(feetFromTarget);

        }
    }

    public double getHoodAngle() {
        return getHoodAngle((Robot.getMetersFromTarget()));
    }

    public void setHoodDutyCycle(double dc) {
        mainMotor.set(dc);
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
        mainMotor.setPosition(pos);
    }

    public void setHoodPositionAuto(double pos) {
        mainMotor.setPosition(pos);
        isHomed = true;
    }

    public void setMM() {
        ShouldMM = (!ShouldMM);
    }

    public void setMM(boolean MM) {
        ShouldMM = MM;
    }

    public double getStatorCurrent() {
        return mainMotor.getTorqueCurrent().getValueAsDouble();

    }

    public void setMotionMagicTarget(double pos) {
        hoodTargetPosition.Position = pos;
    }

    public void setMotionMagic() {

        if (ShouldMM) {

            if (Math.abs(mainMotor.getPosition().getValueAsDouble() - hoodTargetPosition.Position) < 0.04) {
                mainMotor.set(0);
            } else {
                if (hoodTargetPosition.Position > 0.8) {
                    hoodTargetPosition.Position = .8;
                } else if (hoodTargetPosition.Position < 0) {
                    hoodTargetPosition.Position = 0;

                }
                mainMotor.setControl(hoodTargetPosition);

            }
        }

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood/HoodTargetPosition", hoodTargetPosition.Position);
        SmartDashboard.putNumber("Hood/Hood pos", mainMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Stator Current", getStatorCurrent());
        SmartDashboard.putBoolean("Hood/manual", ShouldMM);
        if (mainMotor.hasResetOccurred()) {
            isHomed = false;
        }

        if (!isHomed) {
            CommandScheduler.getInstance().schedule(new HomeHood().withTimeout(4));
        }

    }

}
