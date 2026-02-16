package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
@Logged
public class TurretShooter extends SubsystemBase{
    private TalonFX shooterMotor;
    private MotionMagicVelocityVoltage shooterTargetVelocity;

    public static TurretShooter instance;

    public static TurretShooter getInstance(){
        if (instance == null) {
            instance = new TurretShooter();
        }
        return instance;
    }

    public TurretShooter(){
        shooterMotor =new TalonFX(Constants.TurretConstants.pepperMotorID);

        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

        shooterConfig.Slot0.kP = Constants.TurretConstants.kShooterMotorP; // An error of 1 rotation results in 2.4 V output
        shooterConfig.Slot0.kI = Constants.TurretConstants.kShooterMotorI; // no output for integrated error
        shooterConfig.Slot0.kD = Constants.TurretConstants.kShooterMotorD;
        shooterConfig.Slot0.kA = Constants.TurretConstants.kShooterMotorA;
        shooterConfig.Slot0.kV = Constants.TurretConstants.kShooterMotorV;
        // set Motion Magic settings
        var motionMagicConfigs = shooterConfig.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = Constants.TurretConstants.shooterMotionMagicAcceleration; 
        motionMagicConfigs.MotionMagicJerk = Constants.TurretConstants.shooterMotionMagicJerk; 

        // Apply the PID configurations to the shooter motor
        shooterMotor.getConfigurator().apply(shooterConfig);
        setShooterVelocity(0);

    }

    public void setShooterVelocity(double velocity) {
        // Set the shooter motor to the specified duty
        shooterTargetVelocity=new MotionMagicVelocityVoltage(velocity);
        
    }
    public double currentShooterVelocity() {
        // Get the current velocity of the shooter motor
        return shooterMotor.get(); // Returns the current velocity of the shooter motor
    }

    public boolean shooterAtTargetVelocity(){
        boolean atTarget = Math.abs(currentShooterVelocity() - shooterTargetVelocity.Velocity) < Constants.TurretConstants.kShooterVelocityTolerance;
        return atTarget; // Check if the current velocity is within a small threshold of the target velocity

    }

    @Override
    public void periodic() {
        shooterMotor.setControl(shooterTargetVelocity);
    }
    
}
