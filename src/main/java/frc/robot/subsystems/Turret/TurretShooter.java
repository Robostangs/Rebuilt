package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

        shooterConfig.Slot0.kP = Constants.TurretConstants.kShooterMotorP; 
        shooterConfig.Slot0.kI = Constants.TurretConstants.kShooterMotorI; 
        shooterConfig.Slot0.kD = Constants.TurretConstants.kShooterMotorD;
        shooterConfig.Slot0.kA = Constants.TurretConstants.kShooterMotorA;
        shooterConfig.Slot0.kV = Constants.TurretConstants.kShooterMotorV;
        shooterConfig.Slot0.kS = Constants.TurretConstants.kShooterMotorS;
        // set Motion Magic settings
        var motionMagicConfigs = shooterConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.TurretConstants.shooterMotionMagicCruiseVelocity; 
        motionMagicConfigs.MotionMagicAcceleration = Constants.TurretConstants.shooterMotionMagicAcceleration; 
        shooterTargetVelocity=new MotionMagicVelocityVoltage(0);

        // Apply the PID configurations to the shooter motor
        shooterMotor.getConfigurator().apply(shooterConfig);

    }

   
    public double currentShooterVelocity() {
        // Get the current velocity of the shooter motor
        return shooterMotor.get(); // Returns the current velocity of the shooter motor
    }

    public boolean shooterAtTargetVelocity(){
        boolean atTarget = Math.abs(currentShooterVelocity() - shooterTargetVelocity.Velocity) < Constants.TurretConstants.kShooterVelocityTolerance;
        return atTarget; // Check if the current velocity is within a small threshold of the target velocity

    }
    public void setShooterRPS(double rps){
        shooterTargetVelocity.Velocity = rps;


    }
    @Override
    public void periodic() {
        shooterMotor.setControl(shooterTargetVelocity);
        SmartDashboard.putNumber("Shooter/velocity",shooterMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/target",shooterTargetVelocity.Velocity);
    }
    
}
