package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
@Logged
public class Hood extends SubsystemBase {
    TalonFXS hoodMotor;
    public static Hood mInstance;
    DigitalInput limitSwitch;

    public Hood() {
        hoodMotor = new TalonFXS(Constants.HoodConstants.hoodMotorID);
        limitSwitch = new DigitalInput(Constants.HoodConstants.kLimitSwitchId);
    }

    public void configureHoodMotor() {
        TalonFXSConfiguration hoodMotorConfig = new TalonFXSConfiguration();

        hoodMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;

        hoodMotor.getConfigurator().apply(hoodMotorConfig);

        hoodMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;

        hoodMotor.getConfigurator().apply(hoodMotorConfig);

    }

    public double getHoodAngle(Pose2d currentPose){
        //TODO acc fill this out later
        return currentPose.getX();
    }

    

    public void setHoodDutyCycle(double dc) {
        hoodMotor.set(dc);
    }

    public static Hood getInstance() {
        if (mInstance == null) {
            mInstance = new Hood();
        }
        return mInstance;
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Hood/status", status);
    }

    public void setHoodPosition(double pos) {
        hoodMotor.setPosition(pos);
    }
    
    public boolean getLimitSwitch(){
        return limitSwitch.get();
    }

}
