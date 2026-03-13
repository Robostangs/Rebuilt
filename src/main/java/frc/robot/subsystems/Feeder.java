package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

// @Logged
public class Feeder extends SubsystemBase {
    TalonFX feederMotor;
    TalonFXConfiguration feedConfigurator;
    DigitalInput fuelSensor;
    public static boolean force = false;


    private static Feeder instance;

    public Feeder() {
        feederMotor = new TalonFX(Constants.FeederConstants.feederMotorID);
        fuelSensor = new DigitalInput(Constants.FeederConstants.fuelSensorID);
    }

    public void configureFeeder(){
        feedConfigurator = new TalonFXConfiguration();
        feedConfigurator.CurrentLimits.SupplyCurrentLimit = Constants.FeederConstants.feederSupplyLimit;
        
        
        feederMotor.getConfigurator().apply(feedConfigurator);
    }


    public void setDutyCycle(double dutyCycle) {
        feederMotor.set(dutyCycle);
    }

    public void forceTog(){
        force = !force;
    }
    public boolean getfuelSensor(){
        return fuelSensor.get();
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Feeder/status", status);
    }

    public static Feeder getInstance() {
        if (instance == null) {
            instance = new Feeder();
        }
        return instance;
    }
    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Photoelectric sensor", getfuelSensor());
    }
}
