package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.SensorPhaseValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * The turret
 */
public class Turret extends SubsystemBase {

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
    TalonFXS turretMotor;

    MotionMagicTorqueCurrentFOC controlThreeThousand;

    CANcoder turretCANcoder;
    TalonFXConfiguration pepperConfig;

    TalonFXConfiguration happyConfig;

    Pose2d turretPose;

    static boolean isHub = true;
    static boolean manualTarget = false;

    private static Turret mInstance;

    /**
     * The Turret
     */
    public Turret() {
        pepperMotor = new TalonFX(Constants.TurretConstants.pepperMotorID);
        happyMotor = new TalonFX(Constants.TurretConstants.happyMotorID);
        turretMotor = new TalonFXS(Constants.TurretConstants.turretMotorID);
        turretCANcoder = new CANcoder(Constants.TurretConstants.turretCANcoderID);

        turretPose = CommandSwerveDrivetrain.getInstance().getPose().transformBy(
                new Transform2d(
                        new Translation2d(
                                // TODO find these values
                                Constants.TurretConstants.kIronmanToRobotCenterX,
                                Constants.TurretConstants.kIronmanToRobotCenterY),
                        new Rotation2d()));

        controlThreeThousand = new MotionMagicTorqueCurrentFOC(turretMotor.getPosition().getValueAsDouble());

        pepperConfig = new TalonFXConfiguration();
        happyConfig = new TalonFXConfiguration();

        // TODO find supply current limit for each motor
        configureTurretMotor();
        configureTurretCANcoder();
        configurePepper();
        configureHappy();

    }

    public void configureTurretMotor() {
        TalonFXSConfiguration turretMotorConfig = new TalonFXSConfiguration();

        TalonFXSConfiguration oldConfigs = new TalonFXSConfiguration();
        turretMotor.getConfigurator().refresh(oldConfigs);

        turretMotorConfig.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;
        turretMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;

        turretMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.TurretConstants.TurretMotorSupplyCurrentLimit;

        turretMotorConfig.ExternalFeedback.AbsoluteSensorDiscontinuityPoint = 1;
        turretMotorConfig.ExternalFeedback.AbsoluteSensorOffset = oldConfigs.ExternalFeedback.AbsoluteSensorOffset; /*
                                                                                                                     * When
                                                                                                                     * offset
                                                                                                                     * is
                                                                                                                     * set
                                                                                                                     * in
                                                                                                                     * tuner,
                                                                                                                     * it
                                                                                                                     * may
                                                                                                                     * cause
                                                                                                                     * the
                                                                                                                     * sensor
                                                                                                                     * position
                                                                                                                     * to
                                                                                                                     * be
                                                                                                                     * 1
                                                                                                                     * .
                                                                                                                     * 000
                                                                                                                     * not
                                                                                                                     * 0
                                                                                                                     * .
                                                                                                                     * 000,
                                                                                                                     * that
                                                                                                                     * is
                                                                                                                     * fine
                                                                                                                     */
        turretMotorConfig.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.PulseWidth;
        turretMotorConfig.ExternalFeedback.RotorToSensorRatio = Constants.TurretConstants.kTurretPlanetaryRatio;
        turretMotorConfig.ExternalFeedback.SensorToMechanismRatio = Constants.TurretConstants.kTurretRingTeeth
                / Constants.TurretConstants.kTurretMotorTeeth;
        turretMotorConfig.ExternalFeedback.SensorPhase = SensorPhaseValue.Opposed;

        var ironmanMM = turretMotorConfig.Slot0;

        ironmanMM.kV = Constants.TurretConstants.IronmanMotionMagicKv;
        ironmanMM.kA = Constants.TurretConstants.IronmanMotionMagicKa;
        ironmanMM.kS = Constants.TurretConstants.IronmanMotionMagicKS;

        ironmanMM.kP = Constants.TurretConstants.IronmanMotionMagicKP;
        ironmanMM.kI = Constants.TurretConstants.IronmanMotionMagicKI;
        ironmanMM.kD = Constants.TurretConstants.IronmanMotionMagicKD;

        turretMotor.getConfigurator().apply(turretMotorConfig);
    }

    public void configureTurretCANcoder() {
        CANcoderConfiguration turretCANcoderConfig = new CANcoderConfiguration();

        CANcoderConfiguration oldConfigs = new CANcoderConfiguration();
        turretCANcoder.getConfigurator().refresh(oldConfigs);

        turretCANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        turretCANcoderConfig.MagnetSensor.MagnetOffset = oldConfigs.MagnetSensor.MagnetOffset;

        turretCANcoder.getConfigurator().apply(turretCANcoderConfig);
    }

    public void configurePepper() {
        pepperConfig.CurrentLimits.SupplyCurrentLimit = Constants.TurretConstants.CurtisSupplyCurrentLimit;
        pepperMotor.getConfigurator().apply(pepperConfig);
    }

    public void configureHappy() {
        happyConfig.CurrentLimits.SupplyCurrentLimit = Constants.TurretConstants.MicroSupplyCurrentLimit;
        happyMotor.getConfigurator().apply(happyConfig);
    }

    public static Turret getInstance() {
        if (mInstance == null) {
            mInstance = new Turret();
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
    public void setIronmansetpoint(double rotations) {
        controlThreeThousand.Position = rotations;
    }

    public void setTurretSetpointDoubleSupplier(DoubleSupplier doubleSupplier) {
        setIronmansetpoint(doubleSupplier.getAsDouble()/2);
                SmartDashboard.putString("Your a wizard Ron", "Hagrid");


    }

    public void setMotionMagic(){
        turretMotor.setControl(controlThreeThousand);
        SmartDashboard.putString("Your a wizard harry", "Hagrid");
    }
    // get sensor to mechinasm ratio (19:200) and (21:200)

    public double getTurretMotorSensorModPosition() {
        /* Convert mechanism position to sensor position */
        return mathMod1(turretMotor.getPosition().getValueAsDouble()
                * Constants.TurretConstants.kTurretMotorSensorToMechanismRatio);
    }

    public double getTurretCANcoderModPosition() {
        /* Convert mechanism position to sensor position */
        return mathMod1(turretCANcoder.getPosition().getValueAsDouble());
    }

    // Modulo that matches like math and google sheets
    private static double mathMod1(double dividend) {
        double remainder = dividend % 1;
        if (remainder < 0) {
            remainder = remainder + 1;
        }
        return remainder;
    }

    // ** on initilize get small tooth, use raw gear postions, then use formula,
    // result of formula is the set postion we use to set the current to that */

    private double getGearDifference(double theta1, double theta2) {

        double modulo = mathMod1(theta1 - theta2);

        // TODO, make this constant
        return (modulo)
                * ((Constants.TurretConstants.kTurretMotorTeeth * Constants.TurretConstants.kTurretCANcoderTeeth)
                        / (Constants.TurretConstants.kTurretRingTeeth * 2));
    }

    /**
     * @param theta1 small gear position in rotations
     * @param theta2 big gear position in rotations
     * @return big gear position in rotations
     */
    public double getBigGearPosition() {
        double actualMotorSensorModPos = getTurretMotorSensorModPosition();
        double geardiff = getGearDifference(actualMotorSensorModPos, getTurretCANcoderModPosition());

        double correctedGearDiff;
        if (geardiff > 0.5) {
            correctedGearDiff = (geardiff - 1) * Constants.TurretConstants.dumbThing;
        } else {
            correctedGearDiff = geardiff * Constants.TurretConstants.dumbThing;
        }

        // re-correct using mod position of motor sensor
        // Algorithm gives us an estimate which is definitely correct for whole-number
        // rotations of the sensor,
        // but may not be accurate for the mod-position of the sensor.
        // If the algorithm was perfectly accurate, the sensor mod positions shoud not
        // change when reset
        // Update calculated big gear position so that it uses the whole-number from the
        // CRT but the fractional from the motor sensor

        double calculatedMotorSensorPos = correctedGearDiff
                * Constants.TurretConstants.kTurretMotorSensorToMechanismRatio;

        return (Math.round(calculatedMotorSensorPos - actualMotorSensorModPos) + actualMotorSensorModPos)
                / Constants.TurretConstants.kTurretMotorSensorToMechanismRatio;
    }

    public void setTurretRotation(double rotations) {
        turretMotor.setPosition(rotations);
    }

    public static void setManipTarget(boolean hub) {
        manualTarget = true;
        isHub = hub;
    }

    public void updateIronManPose() {
        turretPose = CommandSwerveDrivetrain.getInstance().getPose().transformBy(
                new Transform2d(
                        new Translation2d(
                                // TODO find these values
                                Constants.TurretConstants.kIronmanToRobotCenterX,
                                Constants.TurretConstants.kIronmanToRobotCenterY),
                        // TODO get this to be right, in rotations

                        new Rotation2d(Units.rotationsToRadians(getTurretPosition()))));
    }

    public Rotation2d aimingJohns() {

        Translation2d ironManTranslation = turretPose.getTranslation().minus(
                (isHub ? Constants.FieldConstants.HubPosition.getTranslation()
                        : Constants.FieldConstants.PassTarget.getTranslation()));

        Rotation2d targetAngle = new Rotation2d(Math.atan2(
                ironManTranslation.getY(),
                ironManTranslation.getX()));
        return targetAngle;
    }

    public double aimingJohnsRotaions() {
        if (aimingJohns().getRadians() / (2 * Math.PI) > 0.495 || aimingJohns().getRadians() / (2 * Math.PI) < -0.495) {
            // Find out what to do, prevent shooting?, alert to driver to rotate robot?
            return turretMotor.getPosition().getValueAsDouble();

        } else {
            return aimingJohns().getRadians() / (2 * Math.PI);
        }

    }

    public double getTurretPosition() {
        return turretMotor.getPosition().getValueAsDouble();
    }

    public void setHappyDutyCycle(double speed) {
        happyMotor.set(speed);
    }

    public void targetHubPose() {
        if (Robot.isRed() ? CommandSwerveDrivetrain.getInstance().getPose().getX() > 12.5
                : CommandSwerveDrivetrain.getInstance().getPose().getX() < 4) {
            isHub = true;
        } else {
            isHub = false;
        }
    }

    public void resetPositionWithSunTzu() {
        double oldMotorModPosition = getTurretMotorSensorModPosition();

        double calculatedPosition = getBigGearPosition();

        setTurretRotation(calculatedPosition);

        double newMotorModPosition = getTurretMotorSensorModPosition();

        SmartDashboard.putNumber("Ironman/SunTzuResetError", (newMotorModPosition - oldMotorModPosition));
        SmartDashboard.putString("IF Sun Tzu","You have art of warred");
    }

    public void periodic() {

        updateIronManPose();

        SmartDashboard.putNumber("Ironman/SunTzuPosition", getBigGearPosition());

        double oldMotorModPosition = getTurretMotorSensorModPosition();

        double calculatedPosition = getBigGearPosition();

        double newMotorModPosition = mathMod1(calculatedPosition * Constants.TurretConstants.kTurretRingTeeth
                / Constants.TurretConstants.kTurretMotorTeeth);

        SmartDashboard.putNumber("Ironman/SunTzuResetPotentialError", (newMotorModPosition - oldMotorModPosition));

        SmartDashboard.putNumber("Ironman/TurretMotorSensorModPosition", oldMotorModPosition);
        SmartDashboard.putNumber("Ironman/TurretCANcoderModPosition", getTurretCANcoderModPosition());

        if (!manualTarget) {
            targetHubPose();
        }

        controlThreeThousand.Position = aimingJohnsRotaions();

        turretMotor.setControl(controlThreeThousand);

        SmartDashboard.putNumber("Ironman/Position", turretMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Ironman/Target", controlThreeThousand.Position);

    }

}
