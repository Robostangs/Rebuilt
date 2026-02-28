package frc.robot.subsystems.Turret;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.SensorPhaseValue;
import com.ctre.phoenix6.sim.TalonFXSSimState;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * The turret
 */

@Logged
public class Turret extends SubsystemBase {

    // Shooter motor has its own subsystem

    /**
     * Pivot control
     */
    TalonFXS turretMotor;

    TalonFXSSimState turretMotorSim;

    MotionMagicExpoVoltage turretControl;

    CANcoder turretCANcoder;
    TalonFXConfiguration pepperConfig;

    Pose2d turretPose;

    static boolean isHub = true;
    static boolean manualTarget = false;
    private static Turret mInstance;

    Mechanism2d turretSim;

    boolean isSafeNeg = false;
    Mechanism2d simMain = new Mechanism2d(60, 60);
    MechanismRoot2d simRoot = simMain.getRoot("TurretBase", 30, 30);
    MechanismLigament2d simDisplay = simRoot.append(
            new MechanismLigament2d("TurretArm", 15, 90, 10, new Color8Bit(150, 150, 150)));

    private double turretMotorAbsoluteSensorOffset = 0.0;

    /**
     * The Turret
     */
    // #region Initializers & Config
    public Turret() {
        turretMotor = new TalonFXS(Constants.TurretConstants.turretMotorID);
        turretCANcoder = new CANcoder(Constants.TurretConstants.turretCANcoderID);

        turretPose = CommandSwerveDrivetrain.getInstance().getState().Pose.transformBy(
                new Transform2d(
                        new Translation2d(
                                // TODO find these values
                                Constants.TurretConstants.kIronmanToRobotCenterX,
                                Constants.TurretConstants.kIronmanToRobotCenterY),
                        new Rotation2d()));

        turretControl = new MotionMagicExpoVoltage(0);

        pepperConfig = new TalonFXConfiguration();

        // TODO find supply current limit for each motor
        configureTurretMotor();
        configureTurretCANcoder();
        /**
         * Kv should be doing mos to of the work(volts)
         * find the actual 19 tooth position by getting the raw pulse width and then sub
         * tracting from offset
         * get raw pulse width by making the hz or sum idk look at docs
         * once we have good postition we pid tune-
         */

    }

    public void configureTurretMotorSoftLimit() {
        TalonFXSConfiguration turretMotorConfig = new TalonFXSConfiguration();
        turretMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.TurretConstants.kForwardSoftLimit;
        turretMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        turretMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.TurretConstants.kReverseSoftLimit;
        turretMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        turretMotor.getConfigurator().apply(turretMotorConfig);
    }

    /*
     * Configure the motor for the turret
     * Sets up the current limit, configures the external feedback sensor as a pulse
     * width sensor,
     * and applies pid for motion magic control.
     *
     */

    public void configureTurretMotor() {
        TalonFXSConfiguration turretMotorConfig = new TalonFXSConfiguration();

        TalonFXSConfiguration oldConfigs = new TalonFXSConfiguration();
        turretMotor.getConfigurator().refresh(oldConfigs);
        turretMotorAbsoluteSensorOffset = oldConfigs.ExternalFeedback.AbsoluteSensorOffset;

        turretMotorConfig.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;
        turretMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;

        turretMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.TurretConstants.TurretMotorSupplyCurrentLimit;

        turretMotorConfig.ExternalFeedback.AbsoluteSensorDiscontinuityPoint = 1;
        turretMotorConfig.ExternalFeedback.AbsoluteSensorOffset = oldConfigs.ExternalFeedback.AbsoluteSensorOffset;
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

    /*
     * Configure the turret cancoder
     *
     */

    public void configureTurretCANcoder() {
        CANcoderConfiguration turretCANcoderConfig = new CANcoderConfiguration();

        CANcoderConfiguration oldConfigs = new CANcoderConfiguration();
        turretCANcoder.getConfigurator().refresh(oldConfigs);

        turretCANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        turretCANcoderConfig.MagnetSensor.MagnetOffset = oldConfigs.MagnetSensor.MagnetOffset;

        turretCANcoder.getConfigurator().apply(turretCANcoderConfig);
    }
    // endregion
    // region Utils
    /*
     * Get the current instance of the Turret
     *
     */

    public static Turret getInstance() {
        if (mInstance == null) {
            mInstance = new Turret();
        }
        return mInstance;
    }

    public void runTurretDutyCycle(double dutyCycle) {
        turretMotor.set(dutyCycle);
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Ironman/status", status);
    }

    public boolean isTurretSafeForClimber() {
        double turretPosition = turretMotor.getPosition().getValueAsDouble();
        if (isSafeNeg) {
            return turretPosition <= Constants.TurretConstants.kSafeClimberNeg;
        } else {
            return turretPosition >= Constants.TurretConstants.kSafeClimberPos;
        }
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

    public void trackHub() {
        turretControl.Position = aimingJohns().getRotations();
    }

    public void setMotionMagic() {
        // turretMotor.setControl(controlThreeThousand);
        SmartDashboard.putString("Your a wizard harry", "Hagrid");
    }

    // endregion
    // #region Turret Calcs
    // get sensor to mechinasm ratio (19:200) and (21:200)
    private double getTurretMotorSensorModPosition() {
        /* Convert mechanism position to sensor position */
        return mathMod1(turretMotor.getPosition().getValueAsDouble()
                * Constants.TurretConstants.kTurretMotorSensorToMechanismRatio);
    }

    private void updateTurretSim() {

        Pose2d robotPose = CommandSwerveDrivetrain.getInstance().getState().Pose;
        Rotation2d angle = robotPose.getRotation().plus(aimingJohns());
        simDisplay.setAngle(angle.getDegrees());

        Translation2d turretOffsetFromRobotCenter = new Translation2d(
                Constants.TurretConstants.kIronmanToRobotCenterX,
                Constants.TurretConstants.kIronmanToRobotCenterY);

        // the turret offset by the robot's rotation to get field relative pose
        Translation2d turretFieldPosition = robotPose.getTranslation()
                .plus(turretOffsetFromRobotCenter.rotateBy(robotPose.getRotation()));

        Pose2d turretPose = new Pose2d(turretFieldPosition, angle);

        Robot.teleopField.getObject("Turret Pivot").setPose(turretPose);
        SmartDashboard.putData("Turret Pivot Target", simMain);
    }

    public Pose2d getTurretPose(){
        return turretPose;
    }

    private double getTurretCANcoderModPosition() {
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
    private double getBigGearPosition() {
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
                * Constants.TurretConstants.kTurretRingTeeth / Constants.TurretConstants.kTurretCANcoderTeeth;

        SmartDashboard.putNumber("correctedGearDiff", correctedGearDiff);

        SmartDashboard.putNumber("calculatedCANcoderModPos", mathMod1(calculatedCANcoderPos));

        return (Math.round(calculatedCANcoderPos - actualCANcoderModPos) + actualCANcoderModPos)
                / Constants.TurretConstants.kTurretMotorSensorToMechanismRatio;
    }

    private double getBigGearPositionUsingMotorSensor() {
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

    public static void setManualTarget(boolean manualTarget) {
        Turret.manualTarget = manualTarget;
    }

    public static void setManipTarget(boolean hub) {
        manualTarget = true;
        isHub = hub;
    }

    private Rotation2d aimingJohns() {
        // get current robot pose
        Pose2d robotPose = CommandSwerveDrivetrain.getInstance().getState().Pose;
        

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

    private void updateIronManPose() {
        // Get the robot's pose
        Pose2d robotPose = CommandSwerveDrivetrain.getInstance().getState().Pose;

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

    private void targetHubPose() {
        if (Robot.isRed() ? CommandSwerveDrivetrain.getInstance().getState().Pose.getX() > 12.5
                : CommandSwerveDrivetrain.getInstance().getState().Pose.getX() < 4) {
            isHub = true;
        } else {
            isHub = false;
        }
    }

    public void moveTurretSafe() {

        if (turretMotor.getPosition().getValueAsDouble() > Constants.TurretConstants.kSafeClimberNeg) {
            turretControl.Position = Constants.TurretConstants.kSafeClimberNeg;
            isSafeNeg = true;
        } else if (turretMotor.getPosition().getValueAsDouble() < Constants.TurretConstants.kSafeClimberPos) {
            turretControl.Position = Constants.TurretConstants.kSafeClimberPos;
            isSafeNeg = false;

        }
        manualTarget = true;
    }

    public void resetPositionWithSunTzu() {
        double oldMotorModPosition = getTurretMotorSensorModPosition();

        double calculatedPosition = getBigGearPosition();

        setTurretRotation(calculatedPosition);

        double newMotorModPosition = getTurretMotorSensorModPosition();

        SmartDashboard.putNumber("Ironman/SunTzuResetError", (newMotorModPosition - oldMotorModPosition));
        SmartDashboard.putString("IF Sun Tzu", "You have art of warred");
    }

    // endregion
    // region Logging & Periodic

    private void logTurretThings() {
        SmartDashboard.putNumber("Ironman/SunTzuPosition", getBigGearPosition());

        double oldMotorModPosition = getTurretMotorSensorModPosition();

        double calculatedPosition = getBigGearPosition();

        double newMotorModPosition = mathMod1(calculatedPosition * Constants.TurretConstants.kTurretRingTeeth
                / Constants.TurretConstants.kTurretMotorTeeth);

        SmartDashboard.putNumber("Ironman/SunTzuResetPotentialError", (newMotorModPosition - oldMotorModPosition));

        double calculatedPosition2 = getBigGearPositionUsingMotorSensor();

        double newMotorModPosition2 = mathMod1(calculatedPosition2 * Constants.TurretConstants.kTurretRingTeeth
                / Constants.TurretConstants.kTurretMotorTeeth);

        SmartDashboard.putNumber("Ironman/SunTzuResetPotentialErrorUsingMotor",
                (newMotorModPosition2 - oldMotorModPosition));

        SmartDashboard.putNumber("Ironman/TurretMotorSensorModPosition", oldMotorModPosition);
        SmartDashboard.putNumber("Ironman/TurretCANcoderModPosition", getTurretCANcoderModPosition());

        SmartDashboard.putNumber("Velocity Turret Piviot", turretMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Turret motor voltage", turretMotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Ironman/Position", turretMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Ironman/Target", turretControl.Position);

        SmartDashboard.putNumber("19Gear-offset, true absolute pos",
                (turretMotor.getRawPulseWidthPosition().getValueAsDouble() - 0.33740234375));
    }

    @Override
    public void periodic() {
        // updateIronManPose();
        updateTurretSim();

        if (!manualTarget && CommandSwerveDrivetrain.getInstance().getState().Pose != null) {
            targetHubPose();
        }
        if (isHub) {
            trackHub();
        }

        turretMotor.setControl(turretControl);

    }
    // endregion
}
