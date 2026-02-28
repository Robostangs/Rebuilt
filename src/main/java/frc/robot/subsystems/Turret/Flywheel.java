package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class Flywheel extends SubsystemBase {
    private TalonFX flyWheel;
    private VelocityVoltage shooterTargetVelocity;

    public static Flywheel instance;
    public static boolean shouldShoot;
    Alert hotAlert = new Alert("Flywheel Motor is HOT", AlertType.kError);
    public static Flywheel getInstance(){

    public static Flywheel getInstance() {
        if (instance == null) {
            instance = new Flywheel();
        }
        return instance;
    }

    public Flywheel() {

        shouldShoot = false;
        flyWheel =new TalonFX(Constants.TurretConstants.shooterMotorID);
        shooterTargetVelocity = new VelocityVoltage(0);
        configureShooterMotor();


    }

    public void configureShooterMotor() {
        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();


        
        flywheelConfig.CurrentLimits.SupplyCurrentLimit = Constants.TurretConstants.kShooterSupplyCurrentLimit;
        flywheelConfig.Slot0.kP = Constants.FlywheelConstants.kShooterMotorP;
        flywheelConfig.Slot0.kI = Constants.FlywheelConstants.kShooterMotorI;
        flywheelConfig.Slot0.kD = Constants.FlywheelConstants.kShooterMotorD;
        flywheelConfig.Slot0.kA = Constants.FlywheelConstants.kShooterMotorA;
        flywheelConfig.Slot0.kV = Constants.FlywheelConstants.kShooterMotorV;
        flywheelConfig.Slot0.kS = Constants.FlywheelConstants.kShooterMotorS;
        flywheelConfig.Voltage.PeakReverseVoltage = Constants.FlywheelConstants.kPeakReverseOutput;

        flywheelConfig.CurrentLimits.StatorCurrentLimit = Constants.FlywheelConstants.kShooterMotorStatorCurrentLimit;

        // Apply the PID configurations to the shooter motor
        flyWheel.getConfigurator().apply(flywheelConfig);

    }

    public double currentShooterVelocity() {
        // Get the current velocity of the shooter motor
        return flyWheel.get(); // Returns the current velocity of the shooter motor
    }

    public void setShouldShoot(boolean shoot) {
        shouldShoot = shoot;
    }

    public boolean shooterAtTargetVelocity() {
        boolean atTarget = Math.abs(currentShooterVelocity()
                - shooterTargetVelocity.Velocity) < Constants.TurretConstants.kShooterVelocityTolerance;
        return atTarget; // Check if the current velocity is within a small threshold of the target
                         // velocity

    }

    public void setShooterRPS(double rps) {
        shooterTargetVelocity.Velocity = rps;
    }
    public void setTargetVelocity(double targetRps){
            shooterTargetVelocity.Velocity = targetRps;
    }
    public void checkIfMotorHot(){
        boolean isMotorHot = flyWheel.getAncillaryDeviceTemp().getValueAsDouble()>=45;
        hotAlert.set(isMotorHot);
        SmartDashboard.putBoolean("Shoooter/isMotorHot",isMotorHot);
    }

    public double getTargetVelocity(double MetersFromHub){
        double feetFromHub = Units.metersToFeet(MetersFromHub);
        double targetVelocity =1.77*feetFromHub+54.3;
        return targetVelocity;
    }


    @Override
    public void periodic() {
        // shooterTargetVelocity.Velocity = SmartDashboard.getNumber("Flywheel RPM target", 0);
        flyWheel.setControl(shooterTargetVelocity);
        SmartDashboard.putNumber("Shooter/velocity",flyWheel.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/target",shooterTargetVelocity.Velocity);
        SmartDashboard.putNumber("Shoooter/temperature",flyWheel.getAncillaryDeviceTemp().getValueAsDouble());
        
        
    }

    public void postStatus(String status) {
        SmartDashboard.putNumber("Shooter/velocity", flyWheel.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/target", shooterTargetVelocity.Velocity);
    }

    public void postStatus(String status) {

        SmartDashboard.putString("TurretShooter/Status", status);
    }

}
