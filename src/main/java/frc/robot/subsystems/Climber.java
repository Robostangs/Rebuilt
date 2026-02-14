package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    public boolean isStowed;
    private static Climber mInstance;
    TalonFX climberMotor;
    TalonFXConfiguration climberConfig;
    PositionVoltage climberControl;
     TalonFXSConfiguration climberPiviotConfig;

    TalonFXS climberPivot;
    PositionVoltage climberPivotControl;

    
    

    public Climber() {
        climberMotor = new TalonFX(Constants.ClimberConstants.kClimberMotorID);
        climberControl = new PositionVoltage(climberMotor.getPosition().getValueAsDouble()).withSlot(0);

        climberPivot = new TalonFXS(Constants.ClimberConstants.kClimberMotorID);
        isStowed = true;
        
    }

    public void configureClimber() {
        climberConfig = new TalonFXConfiguration();

        climberConfig.CurrentLimits.SupplyCurrentLimit = Constants.ClimberConstants.kClimberCurrentLimit;
        climberConfig.Slot0.kP = Constants.ClimberConstants.kClimberKP;
        climberConfig.Slot0.kI = Constants.ClimberConstants.kClimberKI;
        climberConfig.Slot0.kD = Constants.ClimberConstants.kClimberKD;

        climberMotor.getConfigurator().apply(climberConfig);

    }

    public void configureClimberPiviot() {
        climberPiviotConfig = new TalonFXSConfiguration();

        climberPiviotConfig.CurrentLimits.SupplyCurrentLimit = Constants.ClimberConstants.kClimberPiviotCurrentLimit;
        climberConfig.Slot0.kP = Constants.ClimberConstants.kClimberKP;
        climberConfig.Slot0.kI = Constants.ClimberConstants.kClimberKI;
        climberConfig.Slot0.kD = Constants.ClimberConstants.kClimberKD;
        climberPiviotConfig.Commutation.MotorArrangement = Constants.ClimberConstants.kClimberPiviotExternalFeedbackSource;

        climberPivot.getConfigurator().apply(climberPiviotConfig);
    }

    public void setClimberPosition(double rotations) {
        climberControl.Position = rotations;
    }   

    public void setClimberPiviotPosition(double rotations) {
        climberPivotControl.Position = rotations;
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

    public boolean isStowed(){
        if (isStowed){
            return true;
        }
        else{
            return false;
        }
    }

    public boolean isPivotAtTarget() {
        //does the 0.04 make sense idk
    //also the condition lowkey prob doesnt work
    //TODO: account for the degrees of pivot position that dont work (collides w turret)
        if (Math.abs(climberPivot.getPosition().getValueAsDouble()) < 0.04) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void periodic() {
        climberMotor.setControl(climberControl);
        climberPivot.setControl(climberPivotControl);
    }


    
}
