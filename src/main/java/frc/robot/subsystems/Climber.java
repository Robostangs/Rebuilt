package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private static Climber mInstance;
    TalonFX climberMotor;
    TalonFXConfiguration climberConfig;
    PositionVoltage climberControl;
    

    public Climber() {
        climberMotor = new TalonFX(Constants.ClimberConstants.kClimberMotorID);
        climberControl = new PositionVoltage(climberMotor.getPosition().getValueAsDouble()).withSlot(0);
    }

    public void configureClimber() {
        climberConfig = new TalonFXConfiguration();

        climberConfig.CurrentLimits.SupplyCurrentLimit = Constants.ClimberConstants.kClimberCurrentLimit;
        climberConfig.Slot0.kP = Constants.ClimberConstants.kClimberKP;
        climberConfig.Slot0.kI = Constants.ClimberConstants.kClimberKI;
        climberConfig.Slot0.kD = Constants.ClimberConstants.kClimberKD;

        climberMotor.getConfigurator().apply(climberConfig);

    }

    public void setClimberPosition(double rotations) {
        climberControl.Position = rotations;
    }   


    public static Climber getInstance() {
        if (mInstance == null) {
            mInstance = new Climber();
        }
        return mInstance;
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Climber/Status", status);
    }

    @Override
    public void periodic() {
        climberMotor.setControl(climberControl);
    }


    
}
