
package frc.robot.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Stable.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionStangs {
    private PhotonCamera camera;
    PhotonPoseEstimator ppe;
    private Pose2d currentPose;
    private double bigAmbiuity;
    private boolean isLL;
    private double timestampSeconds;

    private String CameraName;

    public VisionStangs(String LLname, Transform3d robotToCam) {
        camera = new PhotonCamera(LLname);
        this.CameraName = LLname;
        ppe = new PhotonPoseEstimator(Constants.kAprilTagFieldLayout, robotToCam);
        bigAmbiuity = -1;
        isLL = false;
    }

    public VisionStangs(String LLname) {
        this.CameraName = LLname;
        bigAmbiuity = -1;
        isLL = true;
    }

    private void Photonperiodic() {

        bigAmbiuity = 0;

        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {

            visionEst = ppe.estimateCoprocMultiTagPose(result);

            for (var target : result.targets) {
                if (target.poseAmbiguity > bigAmbiuity) {
                    bigAmbiuity = target.poseAmbiguity;
                }
            }

            SmartDashboard.putNumber("Vision/" + camera.getName() + " biggest ambiguity", bigAmbiuity);
        }

        if (bigAmbiuity < Constants.VisionConstants.kPhotonAmbiguityThres) {
            // Only use the estimate if present and valid
            visionEst.ifPresent(est -> {
                var pose2d = est.estimatedPose != null ? est.estimatedPose.toPose2d() : null;

                if (pose2d != null) {
                    timestampSeconds = est.timestampSeconds;
                    Robot.teleopField.getObject(camera.getName() + " Pose").setPose(pose2d);
                    currentPose = pose2d;
                }
            });

        }

    }

    private void LLperiodic() {

        bigAmbiuity = 0;

        /*
         * Only seed ll4 with external yaw angle if not rotating fast, otherwise use
         * internal imu with mode 2
         */

        if (DriverStation.isDisabled()) {
            LimelightHelpers.SetIMUMode(CameraName, 1);
        } else {

            LimelightHelpers.SetRobotOrientation(CameraName,
                    CommandSwerveDrivetrain.getInstance().getState().Pose.getRotation().getDegrees(), 
                    CommandSwerveDrivetrain.getInstance().getPigeon2().getAngularVelocityXWorld().getValueAsDouble(),
                    0d,
                    0d,
                    0d,
                    0d);

            LimelightHelpers.SetIMUMode(CameraName, 4);

        }

        LimelightHelpers.SetIMUAssistAlpha(CameraName, 0.4);

        if (DriverStation.isDisabled()) {
            NetworkTableInstance.getDefault().getTable(CameraName)
                    .getEntry("throttle-set").setNumber(200);
        } else {
            NetworkTableInstance.getDefault().getTable(CameraName)
                    .getEntry("throttle-set").setNumber(0);
        }
        if (!Robot.isSimulation()) {

            LimelightHelpers.PoseEstimate fourPoseEsti;
            if (DriverStation.isDisabled()) {

                fourPoseEsti = LimelightHelpers.getBotPoseEstimate_wpiBlue(CameraName);
                SmartDashboard.putString("Vision/" + CameraName + "MT 1 or 2", "1 BC DISAB:ED");

            }
            else if
            (CommandSwerveDrivetrain.getInstance().getPigeon2().getAngularVelocityZWorld()
            .getValueAsDouble() < Constants.VisionConstants.kVisionAngularThreshold) {

            fourPoseEsti = LimelightHelpers
            .getBotPoseEstimate_wpiBlue(CameraName);

            SmartDashboard.putString("Vision/"+CameraName+"MT 1 or 2", "1");

            }
            else {
                fourPoseEsti = LimelightHelpers
                        .getBotPoseEstimate_wpiBlue(CameraName);
                SmartDashboard.putString("Vision/" + CameraName + "MT 1 or 2", "2 BC NOT DISABELED");

            }
            if (isPosegoo(fourPoseEsti)) {
                this.currentPose = fourPoseEsti.pose;
                this.timestampSeconds = fourPoseEsti.timestampSeconds;
                Robot.teleopField.getObject(CameraName + " Pose LL").setPose(fourPoseEsti.pose);
                for (LimelightHelpers.RawFiducial fiducial : fourPoseEsti.rawFiducials) {
                    if (fiducial.ambiguity > bigAmbiuity) {
                        bigAmbiuity = fiducial.ambiguity;
                    }
                }
                SmartDashboard.putNumber("Vision/" + CameraName + " biggest ambiguity", bigAmbiuity);

            }
        }
    }

    public double getTimeStamp() {
        return timestampSeconds;
    }

    public double getAmbiguity() {
        return bigAmbiuity;
    }

    public Pose2d getPose() {
        return currentPose;
    }

    public String getName() {
        return CameraName;
    }

    // TODO plot stds and make standard deviations based on this input
    public double[] getLLstds() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("stddevs")
                .getDoubleArray(new double[12]);
    }

    public boolean isLL() {
        return isLL;
    }

    private boolean isPosegoo(LimelightHelpers.PoseEstimate yoPoseEsti) {
        if (Robot.isSimulation()) {
            SmartDashboard.putString("Vision"+CameraName+"/LL4_RejectReason", "Simulation mode");
            return false;
        }
        if (yoPoseEsti == null) {
            SmartDashboard.putString("Vision"+CameraName+"/LL4_RejectReason", "Pose null");
            return false;
        }
        if (yoPoseEsti.tagCount < 1) {
            SmartDashboard.putString("Vision"+CameraName+"/LL4_RejectReason", "Not enough tags: " + yoPoseEsti.tagCount);
            return false;
        }
        if (yoPoseEsti.avgTagDist > Constants.VisionConstants.kLLDisThres) {
                        SmartDashboard.putString("Vision"+CameraName+"/LL4_RejectReason", "Too far, Dis is: "+ yoPoseEsti.avgTagDist);

            return false;
        }

        SmartDashboard.putNumber("Vision"+CameraName+"/Avg Dis"+ CameraName, yoPoseEsti.avgTagDist);

        if (!RobotContainer.useVision) {
            SmartDashboard.putString("Vision"+CameraName+"/LL4_RejectReason"+CameraName, "useVision disabled");
        } else {
            SmartDashboard.putString("Vision"+CameraName+"/LL4_RejectReason", "Accepted");
        }

        return RobotContainer.useVision;
    }

    public void periodic() {
        if (isLL) {
            LLperiodic();
        } else {
            Photonperiodic();
        }
    }
}