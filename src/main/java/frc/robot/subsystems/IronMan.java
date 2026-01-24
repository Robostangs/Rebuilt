package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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
    TalonFXS ironmanMotor;

    MotionMagicTorqueCurrentFOC controlThreeThousand;

    CANcoder harleyCANcoder;
    CANcoderConfiguration harleyConfig;

    CANcoder yinsenCANcoder;
    CANcoderConfiguration yinsenConfig;

    TalonFXSConfiguration ironmanConfig;

    TalonFXConfiguration pepperConfig;

    TalonFXConfiguration happyConfig;

     
    
    

    Pose2d ironManPose;

    static boolean isHub = true;
    static boolean manualTarget = false;

    private static IronMan mInstance;

    /**
     * The Turret
     */
    public IronMan() {
        pepperMotor = new TalonFX(Constants.IronManConstants.pepperMotorID);
        happyMotor = new TalonFX(Constants.IronManConstants.happyMotorID);
        ironmanMotor = new TalonFXS(Constants.IronManConstants.ironmanMotorID);

        ironManPose = CommandSwerveDrivetrain.getInstance().getPose().transformBy(
                new Transform2d(
                        new Translation2d(
                                // TODO find these values
                                Constants.IronManConstants.kIronmanToRobotCenterX,
                                Constants.IronManConstants.kIronmanToRobotCenterY),
                        new Rotation2d()));

        controlThreeThousand = new MotionMagicTorqueCurrentFOC(ironmanMotor.getPosition().getValueAsDouble());

        harleyConfig = new CANcoderConfiguration();

        ironmanConfig = new TalonFXSConfiguration();
        pepperConfig = new TalonFXConfiguration();
        happyConfig = new TalonFXConfiguration();

        // TODO find supply current limit for each motor
        configureIronman();
        configurePepper();
        configureHappy();


    }

    public void configureIronman() {
        ironmanConfig.ExternalFeedback.SensorToMechanismRatio = 19 / 200;
        ironmanConfig.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.RemoteCANcoder;
        ironmanConfig.CurrentLimits.SupplyCurrentLimit = Constants.IronManConstants.IronManSupplyCurrentLimit;
        ironmanConfig.ExternalFeedback.FeedbackRemoteSensorID = harleyCANcoder.getDeviceID();
        ironmanConfig.ExternalFeedback.RotorToSensorRatio= Constants.IronManConstants.kRotorToSensorRatio;
        


        var ironmanMM = ironmanConfig.Slot0;

        ironmanMM.kV = Constants.IronManConstants.IronmanMotionMagicKv;
        ironmanMM.kA = Constants.IronManConstants.IronmanMotionMagicKa;
        ironmanMM.kS = Constants.IronManConstants.IronmanMotionMagicKS;

        ironmanMM.kP = Constants.IronManConstants.IronmanMotionMagicKP;
        ironmanMM.kI = Constants.IronManConstants.IronmanMotionMagicKI;
        ironmanMM.kD = Constants.IronManConstants.IronmanMotionMagicKD;


        ironmanMotor.getConfigurator().apply(ironmanConfig);
    }

    public void configurePepper() {
        pepperConfig.CurrentLimits.SupplyCurrentLimit = Constants.IronManConstants.CurtisSupplyCurrentLimit;
        pepperMotor.getConfigurator().apply(pepperConfig);
    }

    public void configureHappy() {
        happyConfig.CurrentLimits.SupplyCurrentLimit = Constants.IronManConstants.MicroSupplyCurrentLimit;
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
    public void setIronmansetpoint(double rotations) {
        controlThreeThousand.Position = rotations;
    }
    // get sensor to mechinasm ratio (19:200) and (21:200)

    // ** on initilize get small tooth, use raw gear postions, then use formula,
    // result of formula is the set postion we use to set the current to that */

    private double getGearDifference(double theta1, double theta2) {

        double remainder = (theta1 - theta2) % 1;
        if (remainder < 0) {
            remainder = remainder + 1;

        }

        // TODO, make this constant
        return (remainder)
                * ((Constants.IronManConstants.kHarleyTeeth * Constants.IronManConstants.kYinsenTeeth)
                        / (Constants.IronManConstants.kObadiahGearteeth * 2));
    }

    /**
     * @param theta1 small gear position in rotations
     * @param theta2 big gear position in rotations
     * @return big gear position in rotations
     */
    public double getBigGearPosition(double theta1, double theta2) {

        double geardiff = getGearDifference(theta1, theta2);

        if (geardiff > 0.5) {
            return (geardiff - 1) * Constants.IronManConstants.dumbThing;
        } else {
            return geardiff * Constants.IronManConstants.dumbThing;
        }

    }

    public void setIronmanRotation(double rotations) {
        ironManPose.transformBy(new Transform2d(0, 0, new Rotation2d(Units.rotationsToRadians(rotations))));
    }

    public double getRawGearPositionHarley() {
        return harleyCANcoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getRawGearPositionYinsen() {
        return yinsenCANcoder.getAbsolutePosition().getValueAsDouble();
    }

    public static void setManipTarget(boolean hub) {
        manualTarget = true;
        isHub = hub;
    }

    public void updateIronManPose(){
        CommandSwerveDrivetrain.getInstance().getPose().transformBy(
                new Transform2d(
                        new Translation2d(
                                // TODO find these values
                                Constants.IronManConstants.kIronmanToRobotCenterX,
                                Constants.IronManConstants.kIronmanToRobotCenterY),
                                        // TODO get this to be right, in rotations

                       new Rotation2d(Units.rotationsToRadians(IronManPosition()))));
    }

    public Rotation2d aimingJohns() {

        Translation2d ironManTranslation = ironManPose.getTranslation().minus(
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
            return ironmanMotor.getPosition().getValueAsDouble();

        } else {
            return aimingJohns().getRadians() / (2 * Math.PI);
        }

    }

    public double IronManPosition() {
        return ironmanMotor.getPosition().getValueAsDouble();
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

    public void periodic() {

        updateIronManPose();




        if(!manualTarget){
            targetHubPose();
        }
      
        controlThreeThousand.Position = aimingJohnsRotaions();

        ironmanMotor.setControl(controlThreeThousand);


        SmartDashboard.putNumber("Ironman/Position", ironmanMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Ironman/Target", controlThreeThousand.Position);

    }

}
