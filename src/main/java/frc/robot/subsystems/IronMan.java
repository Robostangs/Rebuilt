package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The turret
 */
public class IronMan extends SubsystemBase {

    /**
     * Shooter motor
     */
    TalonFX pepperMotor;

    /**
     * Feeder motor
     */
    TalonFX happyMotor;

    /**
     * Pivot control
     */
    TalonFX ironmanMotor;

    MotionMagicTorqueCurrentFOC controlThreeThousand;

    CANcoder harleyCANcoder;
    CANcoderConfiguration harleyConfig;

    CANcoder yinsenCANcoder;
    CANcoderConfiguration yinsenConfig;

    TalonFXConfiguration ironmanConfig;

    TalonFXConfiguration pepperConfig;

    TalonFXConfiguration happyConfig;

    private static IronMan mInstance;

    /**
     * The Turret
     */
    public IronMan() {
        pepperMotor = new TalonFX(Constants.PunisherConstants.pepperMotorID);
        happyMotor = new TalonFX(Constants.PunisherConstants.happyMotorID);
        ironmanMotor = new TalonFX(Constants.PunisherConstants.ironmanMotorID);

        controlThreeThousand = new MotionMagicTorqueCurrentFOC(ironmanMotor.getPosition().getValueAsDouble());

        harleyCANcoder = new CANcoder(Constants.PunisherConstants.kAmyID);
        yinsenCANcoder = new CANcoder(Constants.PunisherConstants.kBillyID);

        ironmanConfig = new TalonFXConfiguration();
        pepperConfig = new TalonFXConfiguration();
        happyConfig = new TalonFXConfiguration();

        // TODO find supply current limit for each motor
        configureIronman();
        configurePepper();
        configureHappy();



    }

    public void configureIronman() {
        ironmanConfig.Feedback.SensorToMechanismRatio = 19 / 200;
        ironmanConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        ironmanConfig.CurrentLimits.SupplyCurrentLimit = Constants.PunisherConstants.PunisherSupplyCurrentLimit;
        ironmanMotor.getConfigurator().apply(ironmanConfig);
    }

    public void configurePepper() {
        pepperConfig.CurrentLimits.SupplyCurrentLimit = Constants.PunisherConstants.CurtisSupplyCurrentLimit;
        pepperMotor.getConfigurator().apply(pepperConfig);
    }

    public void configureHappy() {
        happyConfig.CurrentLimits.SupplyCurrentLimit = Constants.PunisherConstants.MicroSupplyCurrentLimit;
        happyMotor.getConfigurator().apply(happyConfig);
    }

    public static IronMan getInstance() {
        if (mInstance == null) {
            mInstance = new IronMan();
        }
        return mInstance;
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Ironman/status", status);
    }

    public void setPepperdutycycle(double speed) {
        pepperMotor.set(speed);
    }

    /**
     * @param rotations Set motion magic target position in rotations
     */
    public void setIronmanposition(double rotations) {
        controlThreeThousand.Position = rotations;
    }
    // get sensor to mechinasm ratio (19:200) and (21:200)

    // ** on initilize get small tooth, use raw gear postions, then use formula,
    // result of formula is the set postion we use to set the current to that */

    public double getBigGearposition(double theta1, double theta2) {

        double remainder = (theta1 - theta2) % 1;
        if (remainder < 0) {
            remainder = remainder + 1;

        }

        return (remainder)
                * ((Constants.PunisherConstants.kAmyTeeth * Constants.PunisherConstants.kBillyTeeth)
                        / (Constants.PunisherConstants.kPilgramGearteeth * 2));
    }

    public double getRawGearPositionHarley() {
        return harleyCANcoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getRawGearPositionYinsen() {
        return yinsenCANcoder.getAbsolutePosition().getValueAsDouble();
    }

    public void setHappyDutyCycle(double speed) {
        happyMotor.set(speed);

    }

    public void periodic() {
        SmartDashboard.putNumber("Ironman/Position", ironmanMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Ironman/Target", controlThreeThousand.Position);

        ironmanMotor.setControl(controlThreeThousand);

    }

}
