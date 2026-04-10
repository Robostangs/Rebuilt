
package frc.robot.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
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
    private double bigAmbiguity;
    private boolean isLL;
    private double timestampSeconds;

    private String CameraName;

    public VisionStangs(String LLname, Transform3d robotToCam) {
        camera = new PhotonCamera(LLname);
        this.CameraName = LLname;
        ppe = new PhotonPoseEstimator(Constants.kAprilTagFieldLayout, robotToCam);
        bigAmbiguity = -1;
        isLL = false;
    }

    public VisionStangs(String LLname) {
        this.CameraName = LLname;
        bigAmbiguity = -1;
        isLL = true;

        LimelightHelpers.SetFiducialIDFiltersOverride(LLname, Constants.VisionConstants.goodIDS);
    }

    private void Photonperiodic() {

        bigAmbiguity = 0;

        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {

            visionEst = ppe.estimateCoprocMultiTagPose(result);

            for (var target : result.targets) {
                if (target.poseAmbiguity > bigAmbiguity) {
                    bigAmbiguity = target.poseAmbiguity;
                }
            }

            SmartDashboard.putNumber("Vision/" + camera.getName() + " biggest ambiguity", bigAmbiguity);
        }

        if (bigAmbiguity < Constants.VisionConstants.kPhotonAmbiguityThres) {
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

        bigAmbiguity = 0;

        /*
         * Only seed ll4 with external yaw angle if not rotating fast, otherwise use
         * internal imu with mode 2
         */

        boolean isDisabled = DriverStation.isDisabled();
        CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();

        if (isDisabled) {
            LimelightHelpers.SetIMUMode(CameraName, 1);

            LimelightHelpers.SetThrottle(CameraName, 200);
        } else {

            LimelightHelpers.SetRobotOrientation(CameraName,
                    drivetrain.getState().Pose.getRotation().getDegrees(),
                    drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(), 
                    0d,
                    0d,
                    0d,
                    0d);

            LimelightHelpers.SetIMUMode(CameraName, 4);
            LimelightHelpers.SetThrottle(CameraName, 0);

        }

        if (!Robot.isSimulation()) {

            LimelightHelpers.PoseEstimate fourPoseEsti;
            String mtLabel;
            double yawRate = drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
            boolean isFastJohn = Math.abs(yawRate) > Constants.VisionConstants.kVisionAngularThreshold;

            if (isDisabled) {
                fourPoseEsti = LimelightHelpers.getBotPoseEstimate_wpiBlue(CameraName);
                mtLabel = "MT1 (disabled)";
            } else if (isFastJohn) {
                fourPoseEsti = null;
                mtLabel = "Rejected (too fast)";
            } else {
                fourPoseEsti = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(CameraName);
                mtLabel = "MT2";
            }

            SmartDashboard.putString("Vision/" + CameraName + "/MT Mode ", mtLabel);

            if (isPosegoo(fourPoseEsti)) {
                this.currentPose = fourPoseEsti.pose;
                this.timestampSeconds = fourPoseEsti.timestampSeconds;
                Robot.teleopField.getObject(CameraName + " Pose LL").setPose(fourPoseEsti.pose);

                for (LimelightHelpers.RawFiducial fiducial : fourPoseEsti.rawFiducials) {
                    if (fiducial.ambiguity > bigAmbiguity) {
                        bigAmbiguity = fiducial.ambiguity;
                    }
                }
                SmartDashboard.putNumber("Vision/" + CameraName + "/Biggest Ambiguity", bigAmbiguity);
            } else {
                this.currentPose = null;
            }
        }
    }

    public double getTimeStamp() {
        return timestampSeconds;
    }

    public double getAmbiguity() {
        return bigAmbiguity;
    }

    public Pose2d getPose() {
        return currentPose;
    }

    public String getName() {
        return CameraName;
    }

    public boolean isLL() {
        return isLL;
    }

    private boolean isPosegoo(LimelightHelpers.PoseEstimate yoPoseEsti) {
        if (Robot.isSimulation()) {
            SmartDashboard.putString("Vision" + CameraName + "/LL_RejectReason", "Simulation mode");
            return false;
        }
        if (yoPoseEsti == null) {
            SmartDashboard.putString("Vision" + CameraName + "/LL_RejectReason", "Pose null");
            return false;
        }
        if (yoPoseEsti.tagCount < 2) {
            SmartDashboard.putString("Vision" + CameraName + "/LL_RejectReason",
                    "Not enough tags: " + yoPoseEsti.tagCount);
            return false;
        }
        if (yoPoseEsti.avgTagDist > Constants.VisionConstants.kLLDisThres) {
            SmartDashboard.putString("Vision" + CameraName + "/LL_RejectReason", "Too far: " + yoPoseEsti.avgTagDist);
            return false;
        }

        double maxAmbiguity = 0;
        for (LimelightHelpers.RawFiducial fiducial : yoPoseEsti.rawFiducials) {
            if (fiducial.ambiguity > maxAmbiguity) {
                maxAmbiguity = fiducial.ambiguity;
            }
        }
        bigAmbiguity = maxAmbiguity;
        SmartDashboard.putNumber("Vision/" + CameraName + "/Biggest Ambiguity", bigAmbiguity);

        if (bigAmbiguity > Constants.VisionConstants.kLLAmbiguityThres) {
            SmartDashboard.putString("Vision" + CameraName + "/LL_RejectReason", "High ambiguity: " + bigAmbiguity);
            return false;
        }

        if (!RobotContainer.useVision) {
            SmartDashboard.putString("Vision" + CameraName + "/LL_RejectReason", "useVision disabled");
            return false;
        }

        SmartDashboard.putString("Vision" + CameraName + "/LL_RejectReason", "Accepted");
        return true;
    }

    public void periodic() {
        if (isLL) {
            LLperiodic();
        } else {
            Photonperiodic();
        }
    }
}