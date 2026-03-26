package frc.robot.subsystems.Dumper;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Flywheel extends SubsystemBase {
    TalonFX mainFlywheelMotor;
    TalonFX slaveFlywheelMotor;
    public static Flywheel mInstance;
    public Pose2d targetPose;
    private double adjustVal = 0;
    private VelocityVoltage flywheelTargetVelocity = new VelocityVoltage(0);
    private Follower slaveControl;

    public Flywheel() {
        mainFlywheelMotor = new TalonFX(Constants.FlywheelConstants.kFlywheelMotorID);
        slaveFlywheelMotor = new TalonFX(Constants.FlywheelConstants.kFlywheelSlaveMotorID);
        configureFlywheelMotors();
        slaveControl = new Follower(Constants.FlywheelConstants.kFlywheelMotorID, MotorAlignmentValue.Aligned);
    }

    private void configureFlywheelMotors() {
        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

        flywheelConfig.CurrentLimits.SupplyCurrentLimit = Constants.DumperConstants.kShooterSupplyCurrentLimit;
        flywheelConfig.CurrentLimits.StatorCurrentLimit = Constants.FlywheelConstants.kShooterMotorStatorCurrentLimit;

        flywheelConfig.Slot0.kP = Constants.FlywheelConstants.kShooterMotorP;
        flywheelConfig.Slot0.kI = Constants.FlywheelConstants.kShooterMotorI;
        flywheelConfig.Slot0.kD = Constants.FlywheelConstants.kShooterMotorD;
        flywheelConfig.Slot0.kA = Constants.FlywheelConstants.kShooterMotorA;
        flywheelConfig.Slot0.kV = Constants.FlywheelConstants.kShooterMotorV;
        flywheelConfig.Slot0.kS = Constants.FlywheelConstants.kShooterMotorS;

        flywheelConfig.Voltage.PeakReverseVoltage = Constants.FlywheelConstants.kPeakReverseOutput;

        flywheelTargetVelocity.EnableFOC = false;

        // Apply the PID configurations to the shooter motor
        mainFlywheelMotor.getConfigurator().apply(flywheelConfig);

        TalonFXConfiguration flyWheelSlaveConfig = new TalonFXConfiguration();

        flyWheelSlaveConfig.CurrentLimits.StatorCurrentLimit = Constants.FlywheelConstants.kShooterSlaveMotorStatorCurrentLimit;
        flyWheelSlaveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        slaveFlywheelMotor.getConfigurator().apply(flyWheelSlaveConfig);

    }

    public double getFlywheelTargetVelocity() {

        double feetFromTarget = Units.metersToFeet(Robot.getMetersFromTarget());
        if (Robot.isPassing() && feetFromTarget > Constants.FieldConstants.maxRegressionDistance) {
            // full send if we on opposing alliance and we're past the max actual distance
            // for passing
            return Constants.FlywheelConstants.kVelocityAtOppositeAlliancePass;
        } else {
            // Acc get regression here
            return 2.2 * feetFromTarget + 52;

        }
    }

    public void setFlywheelVelocity(double rps) {
        flywheelTargetVelocity.Velocity = rps + adjustVal;
    }

    public void setFlywheelDC(double dc) {
        mainFlywheelMotor.set(dc);
    }

    public static Flywheel getInstance() {
        if (mInstance == null) {
            mInstance = new Flywheel();
        }
        return mInstance;
    }

    public void adjustRPM(DoubleSupplier adjustment) {
        adjustVal = adjustment.getAsDouble();
    }

    public boolean shooterAtTargetVelocity() {
        double currentVelocity = mainFlywheelMotor.getVelocity().getValueAsDouble();
        double targetVelocity = flywheelTargetVelocity.Velocity;
        return Math.abs(currentVelocity - targetVelocity) <= Constants.FlywheelConstants.kShooterVelocityTolerance;
    }

    @Override
    public void periodic() {
        if (Math.abs(CommandSwerveDrivetrain.getInstance().getState().Speeds.omegaRadiansPerSecond) > 2
                || flywheelTargetVelocity.Velocity == 0) {
            mainFlywheelMotor.set(0);
            slaveFlywheelMotor.set(0);
        } else {
            flywheelTargetVelocity.Velocity = flywheelTargetVelocity.Velocity + adjustVal;
            mainFlywheelMotor.setControl(flywheelTargetVelocity);
            slaveFlywheelMotor.setControl(slaveControl);

        }

    }

    public void postStatus(String status) {
        SmartDashboard.putString("Flywheel/Status", status);
    }

}
