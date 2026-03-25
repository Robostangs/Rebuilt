
// AlignToHub.java
package frc.robot.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.util.FlippingUtil;

public class AlignToHub {

    private static final Translation2d hubPos = !Robot.isRed()
            ? Constants.FieldConstants.HubPosition.getTranslation()
            : FlippingUtil.flipFieldPosition(Constants.FieldConstants.HubPosition.getTranslation());

    private static final double shootRadius = Units.inchesToMeters(60);

    public static Pose2d getTargetPose() {
        Translation2d robotPos = CommandSwerveDrivetrain.getInstance()
                .getState().Pose.getTranslation();

        Translation2d hubToRobot = robotPos.minus(hubPos);

        double dist = hubToRobot.getNorm();

        if (dist < 0.01) {
            hubToRobot = new Translation2d(shootRadius, 0);
            dist = shootRadius;
        }

        Translation2d targetTranslation = hubPos.plus(
                hubToRobot.times(shootRadius / dist));

        Rotation2d facingHub = hubToRobot.getAngle().plus(Rotation2d.k180deg);

        return new Pose2d(targetTranslation, facingHub);
    }

    public static void drawShootingRing() {
        int numPoints = 36;
        List<Pose2d> ringPoses = new ArrayList<>();

        for (int i = 0; i < numPoints; i++) {
            double angleRad = (2 * Math.PI / numPoints) * i;
            Translation2d point = hubPos.plus(
                    new Translation2d(shootRadius, new Rotation2d(angleRad)));
            ringPoses.add(new Pose2d(point, new Rotation2d(angleRad)));
        }

        Robot.teleopField.getObject("Shooting Ring").setPoses(ringPoses);
    }

    public static PathPlannerPath generatePath(Pose2d targetPose) {
        Pose2d currentPose = CommandSwerveDrivetrain.getInstance().getState().Pose;

        Pose2d startPose = new Pose2d(
                currentPose.getX(), currentPose.getY(),
                currentPose.getTranslation()
                        .minus(targetPose.getTranslation())
                        .getAngle()
                        .minus(Rotation2d.k180deg));

        Pose2d endPose = new Pose2d(
                targetPose.getX(), targetPose.getY(),
                targetPose.getRotation().plus(Rotation2d.kCCW_90deg));

        List<Waypoint> waypoints;
        double rotationDiff = Math.abs(
                currentPose.getRotation().minus(targetPose.getRotation()).getDegrees());

        if (rotationDiff < 45) {
            waypoints = PathPlannerPath.waypointsFromPoses(startPose, endPose);
        } else {
            waypoints = PathPlannerPath.waypointsFromPoses(
                    startPose,
                    endPose.transformBy(new Transform2d(
                            Units.inchesToMeters(18), 0, Rotation2d.kZero)),
                    endPose);
        }

        PathConstraints constraints = new PathConstraints(

                Constants.TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.7,

                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared * 0.7,

                Constants.AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared * 0.7,
                Constants.AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared * 0.7);

        PathConstraints endConstraints = new PathConstraints(
                Constants.TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.7,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared * 0.7,
                Constants.AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared * 0.7,
                Constants.AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared * 0.7);

        List<RotationTarget> rotationTargets = new ArrayList<>();
        rotationTargets.add(new RotationTarget(
                0.8,
                endPose.getRotation().plus(Rotation2d.fromDegrees(270))));

        List<ConstraintsZone> zones = new ArrayList<>();
        zones.add(new ConstraintsZone(0.6, 1.0, endConstraints));

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                rotationTargets,
                Collections.emptyList(),
                zones,
                Collections.emptyList(),
                constraints,
                null,
                new GoalEndState(0.0, targetPose.getRotation()),
                false);

        path.preventFlipping = true;
        return path;
    }

    public static Command getAlignToHub() {
        return new DeferredCommand(() -> {
            Pose2d target = getTargetPose();

            Robot.teleopField.getObject("Hub Align Pose").setPose(target);

            return AutoBuilder.followPath(generatePath(target));
        }, Set.of(CommandSwerveDrivetrain.getInstance()));
    }
}
