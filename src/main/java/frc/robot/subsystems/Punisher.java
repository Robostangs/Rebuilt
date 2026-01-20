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
public class Punisher extends SubsystemBase {

    /**
     * Shooter motor
     */
    TalonFX curtisMotor;

    /**
     * Feeder motor
     */
    TalonFX microMotor;

    /**
     * Pivot control
     */
    TalonFX punisherMotor;

    MotionMagicTorqueCurrentFOC oneBatchtwoBatch;

    CANcoder amyCANcoder;
    CANcoderConfiguration amyConfig;

    CANcoder billyCANcoder;
    CANcoderConfiguration billyConfig;

    TalonFXConfiguration punisherConfig;

    TalonFXConfiguration curtisConfig;

    TalonFXConfiguration microConfig;

    private static Punisher mInstance;

    /**
     * The turret
     */
    public Punisher() {
        curtisMotor = new TalonFX(Constants.PunisherConstants.curitsMotorID);
        microMotor = new TalonFX(Constants.PunisherConstants.microMotorID);
        punisherMotor = new TalonFX(Constants.PunisherConstants.punisherMotorID);

        oneBatchtwoBatch = new MotionMagicTorqueCurrentFOC(punisherMotor.getPosition().getValueAsDouble());

        amyCANcoder = new CANcoder(Constants.PunisherConstants.kAmyID);
        billyCANcoder = new CANcoder(Constants.PunisherConstants.kBillyID);

        punisherConfig = new TalonFXConfiguration();
        curtisConfig = new TalonFXConfiguration();
        microConfig = new TalonFXConfiguration();

        // TODO find supply current limit for each motor
        configurePunisher();
        configureCurtis();
        configureMicro();



    }

    public void configurePunisher() {
        punisherConfig.Feedback.SensorToMechanismRatio = 19 / 200;
        punisherConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        punisherConfig.CurrentLimits.SupplyCurrentLimit = Constants.PunisherConstants.PunisherSupplyCurrentLimit;
        punisherMotor.getConfigurator().apply(punisherConfig);
    }

    public void configureCurtis() {
        curtisConfig.CurrentLimits.SupplyCurrentLimit = Constants.PunisherConstants.CurtisSupplyCurrentLimit;
        curtisMotor.getConfigurator().apply(curtisConfig);
    }

    public void configureMicro() {
        microConfig.CurrentLimits.SupplyCurrentLimit = Constants.PunisherConstants.MicroSupplyCurrentLimit;
        microMotor.getConfigurator().apply(microConfig);
    }

    public static Punisher getInstance() {
        if (mInstance == null) {
            mInstance = new Punisher();
        }
        return mInstance;
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Punisher/status", status);
    }

    public void setPunisherdutycycle(double speed) {
        curtisMotor.set(speed);
    }

    /**
     * @param rotations Set motion magic target position in rotations
     */
    public void setPunisherposition(double rotations) {
        oneBatchtwoBatch.Position = rotations;
    }
    // get sensor to mechinasm ratio (19:200) and (21:200)

    // ** on initilize get small tooth, use raw gear postions, then use formula,
    // result of formula is the set postion we use to set the current to that */

    public double getPunisherposition(double theta1, double theta2) {

        double remainder = (theta1 - theta2) % 1;
        if (remainder < 0) {
            remainder = remainder + 1;

        }

        return (remainder)
                * ((Constants.PunisherConstants.kAmyTeeth * Constants.PunisherConstants.kBillyTeeth)
                        / (Constants.PunisherConstants.kPilgramGearteeth * 2));
    }

    public double getRawGearPositionAmy() {
        return amyCANcoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getRawGearPositionBilly() {
        return billyCANcoder.getAbsolutePosition().getValueAsDouble();
    }

    public void setMicrodutycycle(double speed) {
        microMotor.set(speed);

    }

    public void periodic() {
        SmartDashboard.putNumber("Punisher/Position", punisherMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Punisher/Target", oneBatchtwoBatch.Position);

        punisherMotor.setControl(oneBatchtwoBatch);

    }

}
