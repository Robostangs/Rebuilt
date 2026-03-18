package frc.robot.subsystems.Dumper;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Flywheel extends SubsystemBase {
    TalonFX mainFlywheelMotor;
    TalonFX slaveFlywheelMotor;
    public static Flywheel mInstance;
    public Pose2d targetPose;
    private VelocityVoltage flywheelTargetVelocity;
    private double adjustVal = 0;
    private boolean canShoot;

    public Flywheel() {
        canShoot = true;
        mainFlywheelMotor = new TalonFX(Constants.FlywheelConstants.kFlywheelMotorID);
        slaveFlywheelMotor = new TalonFX(Constants.FlywheelConstants.kFlywheelSlaveMotorID);
        configureFlywheelMotors();
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

        //TODO make you have to make configs for slave
        slaveFlywheelMotor.setControl(new Follower(mainFlywheelMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public double getFlywheelTargetVelocity() {

        double feetFromTarget = Units.metersToFeet(Robot.getMetersFromTarget());
        if (Robot.isPassing() && feetFromTarget > Constants.FieldConstants.maxRegressionDistance) {
            // full send if we on opposing alliance and we're past the max actual distance
            // for passing
            return Constants.FlywheelConstants.kVelocityAtOppositeAlliancePass;
        } else {
            // Acc get regression here
            return 50;
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

    public void setCanShoot(boolean shouldShoot) {
        canShoot = shouldShoot;
    }

    @Override
    public void periodic() {
        if (canShoot) {
            mainFlywheelMotor.setControl(flywheelTargetVelocity);
            //TODO you have to set control for slave motor when tou set a control for main motor

        }
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Flywheel/Status", status);
    }

}
