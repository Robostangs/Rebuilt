// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.util.List;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Stable.TimedRobotstangs;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret;

public class Robot extends TimedRobotstangs {
    private Command m_autonomousCommand;
    CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();

    private final RobotContainer m_robotContainer;
    public static ShuffleboardTab disableTab, teleopTab;

    public static Field2d teleopField = new Field2d();
    private static GcStatsCollector gscollect = new GcStatsCollector();
    public static Alert gcAlert = new Alert("MEMORY TWEAKING LOOK INTO TS RN", AlertType.kWarning);

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        SmartDashboard.putData("Field", teleopField);

        disableTab = Shuffleboard.getTab("Disabled");
        teleopTab = Shuffleboard.getTab("Teleop");

        disableTab.add("Set Drivetrain motors to coast",
                new InstantCommand(() -> CommandSwerveDrivetrain.getInstance().setCoastMode(true))
                        .ignoringDisable(true));
        disableTab.add("Set Drivetrain motors to brake",
                new InstantCommand(() -> CommandSwerveDrivetrain.getInstance().setCoastMode(false))
                        .ignoringDisable(true));

        disableTab.add("RESET TO SUN TZUUUUUU",
                new InstantCommand(() -> Turret.getInstance().resetPositionWithSunTzu())
                        .ignoringDisable(true));

        teleopTab.add("Motion Magic", new InstantCommand(() -> Turret.getInstance().setMotionMagic()));
        teleopTab.add("Set turret to setpoint 0 ",
                new InstantCommand(Turret.getInstance().setMotionMagicZero));
        teleopTab.add("Set turret to setpoint 0.3",
                new InstantCommand(Turret.getInstance().setMotionMagicPointThree));

        // disabledTab.add()

    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
        gscollect.update();
        teleopField.setRobotPose(drivetrain.getPose());

    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }

    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }

    public static boolean isRed() {
        if (Robot.isSimulation()) {
            return false;
        }
        if (DriverStation.getAlliance().get() == (Alliance.Red)) {
            return true;
        } else {
            return false;
        }

    }

    private static final class GcStatsCollector {

        private List<GarbageCollectorMXBean> gcBeans = ManagementFactory.getGarbageCollectorMXBeans();
        private MemoryMXBean memBean = ManagementFactory.getMemoryMXBean();
        private final long[] lastTimes = new long[gcBeans.size()];
        private final long[] lastCounts = new long[gcBeans.size()];

        public void update() {
            long accumTime = 0;
            long accumCounts = 0;
            for (int i = 0; i < gcBeans.size(); i++) {
                long gcTime = gcBeans.get(i).getCollectionTime();
                long gcCount = gcBeans.get(i).getCollectionCount();
                accumTime += gcTime - lastTimes[i];
                accumCounts += gcCount - lastCounts[i];

                lastTimes[i] = gcTime;
                lastCounts[i] = gcCount;
            }

            SmartDashboard.putNumber("Memory/GC Time MS", (double) accumTime);
            SmartDashboard.putNumber("Memory/GCCounts", (double) accumCounts);
            SmartDashboard.putNumber("Memory/Usage", (double) memBean.getHeapMemoryUsage().getUsed());

            if (accumTime > (20)) {
                gcAlert.set(true);
            } else {
                gcAlert.set(false);

            }
        }
    }
}