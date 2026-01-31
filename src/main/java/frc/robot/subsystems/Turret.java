package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.SensorPhaseValue;
import com.ctre.phoenix6.sim.TalonFXSSimState;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
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

    TalonFXSSimState turretMotorSim;

    TalonFXS hoodMotor;

    MotionMagicExpoVoltage turretControl;

    CANcoder turretCANcoder;
    TalonFXConfiguration pepperConfig;

    TalonFXConfiguration happyConfig;

    Pose2d turretPose;

    static boolean isHub = true;
    static boolean manualTarget = false;

    private static Turret mInstance;

    Mechanism2d turretSim;

    private double turretMotorAbsoluteSensorOffset = 0.0;

    /**
     * The Turret
     */
    public Turret() {
        pepperMotor = new TalonFX(Constants.TurretConstants.pepperMotorID);
        happyMotor = new TalonFX(Constants.TurretConstants.happyMotorID);
        turretMotor = new TalonFXS(Constants.TurretConstants.turretMotorID);
        turretCANcoder = new CANcoder(Constants.TurretConstants.turretCANcoderID);
        hoodMotor = new TalonFXS(Constants.TurretConstants.hoodMotorID);

        turretPose = CommandSwerveDrivetrain.getInstance().getPose().transformBy(
                new Transform2d(
                        new Translation2d(
                                // TODO find these values
                                Constants.TurretConstants.kIronmanToRobotCenterX,
                                Constants.TurretConstants.kIronmanToRobotCenterY),
                        new Rotation2d()));

        turretControl = new MotionMagicExpoVoltage(0);

        pepperConfig = new TalonFXConfiguration();
        happyConfig = new TalonFXConfiguration();

        // TODO find supply current limit for each motor
        configureTurretMotor();
        configureTurretCANcoder();
        configurePepper();
        configureHappy();
        configureHoodMotor();

        /**
         * Kv should be doing mos to of the work(volts)
         * find the actual 19 tooth position by getting the raw pulse width and then sub
         * tracting from offset
         * get raw pulse width by making the hz or sum idk look at docs
         * once we have good postition we pid tune-
         */

    }

    public void configureTurretMotor() {
        TalonFXSConfiguration turretMotorConfig = new TalonFXSConfiguration();

        TalonFXSConfiguration oldConfigs = new TalonFXSConfiguration();
        turretMotor.getConfigurator().refresh(oldConfigs);
        turretMotorAbsoluteSensorOffset = oldConfigs.ExternalFeedback.AbsoluteSensorOffset;

        turretMotorConfig.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;
        turretMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;

        turretMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.TurretConstants.TurretMotorSupplyCurrentLimit;

        turretMotorConfig.ExternalFeedback.AbsoluteSensorDiscontinuityPoint = 1;
        turretMotorConfig.ExternalFeedback.AbsoluteSensorOffset = turretMotorAbsoluteSensorOffset;

        /*
         * When offset is set in tuner, it may cause the sensor position to be 1.00 not
         * 0.00 that is fine
         *
         */
        turretMotorConfig.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.PulseWidth;
        turretMotorConfig.ExternalFeedback.RotorToSensorRatio = Constants.TurretConstants.kTurretPlanetaryRatio;
        turretMotorConfig.ExternalFeedback.SensorToMechanismRatio = Constants.TurretConstants.kTurretRingTeeth
                / Constants.TurretConstants.kTurretMotorTeeth;
        turretMotorConfig.ExternalFeedback.SensorPhase = SensorPhaseValue.Opposed;

        var ironmanMM = turretMotorConfig.Slot0;

        ironmanMM.kV = Constants.TurretConstants.IronmanMotionMagicKvFF;
        ironmanMM.kA = Constants.TurretConstants.IronmanMotionMagicKaFF;
        ironmanMM.kS = Constants.TurretConstants.IronmanMotionMagicKS;

        ironmanMM.kP = Constants.TurretConstants.IronmanMotionMagicKP;
        ironmanMM.kI = Constants.TurretConstants.IronmanMotionMagicKI;
        ironmanMM.kD = Constants.TurretConstants.IronmanMotionMagicKD;

        turretMotorConfig.MotionMagic.MotionMagicExpo_kV = Constants.TurretConstants.IronmanMotionMagicKv;
        turretMotorConfig.MotionMagic.MotionMagicExpo_kA = Constants.TurretConstants.IronmanMotionMagicKa;

        turretMotor.getRawPulseWidthPosition().setUpdateFrequency(50);

        turretMotor.getConfigurator().apply(turretMotorConfig);
    }

    public void configureHoodMotor() {
        TalonFXSConfiguration hoodMotorConfig = new TalonFXSConfiguration();

        hoodMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;

        hoodMotor.getConfigurator().apply(hoodMotorConfig);

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
        turretControl.Position = rotations;
        SmartDashboard.putString("Setpoint has gotten set", "New setpoint at " + rotations);
    }


    public Runnable setMotionMagicPointThree = () -> {
        turretControl.Position = 0.3;
        // turretMotor.setControl(turretControl);
    };

    public void setTurretSetpointDoubleSupplier(DoubleSupplier doubleSupplier) {
        setIronmansetpoint(doubleSupplier.getAsDouble());

    }

    public void setMotionMagic() {
        // turretMotor.setControl(controlThreeThousand);
        SmartDashboard.putString("Your a wizard harry", "Hagrid");
    }
    // get sensor to mechinasm ratio (19:200) and (21:200)

    public double getTurretMotorSensorModPosition() {
        /* Convert mechanism position to sensor position */
        return mathMod1(turretMotor.getPosition().getValueAsDouble()
                * Constants.TurretConstants.kTurretMotorSensorToMechanismRatio);
    }


    //this is pointless cuz the getposition is the sensor not the rotor
    // public double getTurretMotorSensorModPositon() {
    //     /* because sensor phase is inverted */
    //     return mathMod1(1-(turretMotor.getRawPulseWidthPosition().getValueAsDouble() - turretMotorAbsoluteSensorOffset)); //TODO this offset only updates at rio boot
    // }

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
        double actualCANcoderModPos = getTurretCANcoderModPosition();
        double geardiff = getGearDifference(actualMotorSensorModPos, actualCANcoderModPos);

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

        double calculatedCANcoderPos = correctedGearDiff
                *Constants.TurretConstants.kTurretRingTeeth/Constants.TurretConstants.kTurretCANcoderTeeth;

        SmartDashboard.putNumber("correctedGearDiff", correctedGearDiff);

        SmartDashboard.putNumber("calculatedCANcoderModPos", mathMod1(calculatedCANcoderPos));


        return (Math.round(calculatedCANcoderPos - actualCANcoderModPos) + actualCANcoderModPos)
                / Constants.TurretConstants.kTurretMotorSensorToMechanismRatio;
    }

        public double getBigGearPositionUsingMotorSensor() {
        double actualMotorSensorModPos = getTurretMotorSensorModPosition();
        double turretCANcoderSensorModPos = getTurretCANcoderModPosition();
        double geardiff = getGearDifference(actualMotorSensorModPos, turretCANcoderSensorModPos);

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

        SmartDashboard.putNumber("correctedGearDiff", correctedGearDiff);

        SmartDashboard.putNumber("calculatedMotorSensorPos", calculatedMotorSensorPos);
        SmartDashboard.putNumber("calculatedMotorSensorModPos", mathMod1(calculatedMotorSensorPos));


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

    public Rotation2d aimingJohns() {
        // get current robot pose
        Pose2d robotPose = CommandSwerveDrivetrain.getInstance().getPose();

        // calculate turret position in field space
        Translation2d turretOffset = new Translation2d(
                Constants.TurretConstants.kIronmanToRobotCenterX,
                Constants.TurretConstants.kIronmanToRobotCenterY);

        Translation2d turretFieldPosition = robotPose.getTranslation()
                .plus(turretOffset.rotateBy(robotPose.getRotation()));

        // the vector from turret to target
        Translation2d targetVector = Constants.FieldConstants.HubPosition.getTranslation()
                .minus(turretFieldPosition);

        // angle to target in world frame
        Rotation2d targetAngle = targetVector.getAngle();

        // Convert to turret relative angle
        Rotation2d turretRelativeAngle = targetAngle.minus(robotPose.getRotation());

        SmartDashboard.putNumber("Ironman/TargetAngleRotation", turretRelativeAngle.getRotations());

        return turretRelativeAngle;
    }

    public void updateIronManPose() {
        // Get the robot's pose
        Pose2d robotPose = CommandSwerveDrivetrain.getInstance().getPose();

        // turret's position robot relative
        Translation2d turretOffsetFromRobotCenter = new Translation2d(
                Constants.TurretConstants.kIronmanToRobotCenterX,
                Constants.TurretConstants.kIronmanToRobotCenterY);

        // the turret offset by the robot's rotation to get field relative pose
        Translation2d turretFieldPosition = robotPose.getTranslation()
                .plus(turretOffsetFromRobotCenter.rotateBy(robotPose.getRotation()));

        // The turret's field relative rotation is the robot rotation + turret relative
        // rotation
        Rotation2d turretFieldRotation = robotPose.getRotation()
                .plus(new Rotation2d(Units.rotationsToRadians(turretMotor.getPosition().getValueAsDouble())));

        // Create the turret pose
        turretPose = new Pose2d(turretFieldPosition, turretFieldRotation);
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
        SmartDashboard.putString("IF Sun Tzu", "You have art of warred");
    }

    public void logTurretThings() {
        SmartDashboard.putNumber("Ironman/SunTzuPosition", getBigGearPosition());

        double oldMotorModPosition = getTurretMotorSensorModPosition();

        double calculatedPosition = getBigGearPosition();

        double newMotorModPosition = mathMod1(calculatedPosition * Constants.TurretConstants.kTurretRingTeeth
                / Constants.TurretConstants.kTurretMotorTeeth);

        SmartDashboard.putNumber("Ironman/SunTzuResetPotentialError", (newMotorModPosition - oldMotorModPosition));



                double calculatedPosition2 = getBigGearPositionUsingMotorSensor();

        double newMotorModPosition2 = mathMod1(calculatedPosition2 * Constants.TurretConstants.kTurretRingTeeth
                / Constants.TurretConstants.kTurretMotorTeeth);

        SmartDashboard.putNumber("Ironman/SunTzuResetPotentialErrorUsingMotor", (newMotorModPosition2 - oldMotorModPosition));




        SmartDashboard.putNumber("Ironman/TurretMotorSensorModPosition", oldMotorModPosition);
                SmartDashboard.putNumber("Ironman/TurretCANcoderModPosition", getTurretCANcoderModPosition());


        SmartDashboard.putNumber("Velocity Turret Piviot", turretMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Turret motor voltage", turretMotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Ironman/Position", turretMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Ironman/Target", turretControl.Position);
    }

    @Override
    public void periodic() {

        logTurretThings();
        updateIronManPose();

        if (!manualTarget) {
            targetHubPose();
        }

        // controlThreeThousand.Position = aimingJohnsRotaions();

        turretMotor.setControl(turretControl);

        aimingJohns();

        Robot.teleopField.getObject("Turret").setPose(turretPose);

    }

}
