package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase{
    private TalonFX flyWheel;
    private VelocityVoltage shooterTargetVelocity;

    public static Flywheel instance;
    public static boolean shouldShoot;

    public static Flywheel getInstance(){
        if (instance == null) {
            instance = new Flywheel();
        }
        return instance;
    }

    public Flywheel(){

        shouldShoot = false;
        flyWheel =new TalonFX(Constants.TurretConstants.shooterMotorID);
        shooterTargetVelocity = new VelocityVoltage(0);
        configureShooterMotor();


       
    }

   public void configureShooterMotor(){
         TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

        flywheelConfig.Slot0.kP = Constants.TurretConstants.kShooterMotorP; 
        flywheelConfig.Slot0.kI = Constants.TurretConstants.kShooterMotorI; 
        flywheelConfig.Slot0.kD = Constants.TurretConstants.kShooterMotorD;
        flywheelConfig.Slot0.kA = Constants.TurretConstants.kShooterMotorA;
        flywheelConfig.Slot0.kV = Constants.TurretConstants.kShooterMotorV;
        flywheelConfig.Slot0.kS = Constants.TurretConstants.kShooterMotorS;
        
        flywheelConfig.CurrentLimits.SupplyCurrentLimit = Constants.TurretConstants.kShooterSupplyCurrentLimit;


        // Apply the PID configurations to the shooter motor
        flyWheel.getConfigurator().apply(flywheelConfig);

   }
    public double currentShooterVelocity() {
        // Get the current velocity of the shooter motor
        return flyWheel.get(); // Returns the current velocity of the shooter motor
    }

    public void setShouldShoot(boolean shoot){
        shouldShoot = shoot;
    }

    public boolean shooterAtTargetVelocity(){
        boolean atTarget = Math.abs(currentShooterVelocity() - shooterTargetVelocity.Velocity) < Constants.TurretConstants.kShooterVelocityTolerance;
        return atTarget; // Check if the current velocity is within a small threshold of the target velocity

    }
    public void setShooterRPS(double rps){
        shooterTargetVelocity.Velocity = rps;
    }
    public void setMotionMagic(){
                flyWheel.setControl(shooterTargetVelocity);

    }

    @Override
    public void periodic() {
        // shooterTargetVelocity.Velocity = SmartDashboard.getNumber("Flywheel RPM target", 0);
        // flyWheel.setControl(shooterTargetVelocity);
        SmartDashboard.putNumber("Shooter/velocity",flyWheel.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/target",shooterTargetVelocity.Velocity);
        //TODO: see whether to put ancillary or a diff temp
        SmartDashboard.putNumber("Shoooter/temperature",flyWheel.getAncillaryDeviceTemp().getValueAsDouble());
    }

    public void postStatus(String status) {
        
        SmartDashboard.putString("TurretShooter/Status", status);
    }
    
}
