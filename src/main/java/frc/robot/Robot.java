// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.Console;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
import frc.robot.subsystems.Turret.Turret;
@Logged
public class Robot extends TimedRobotstangs {

    private Command m_autonomousCommand;
    CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    public double debugRobot= 6.7d;
    private final RobotContainer m_robotContainer;
    public static ShuffleboardTab disableTab, teleopTab;

    public static Field2d teleopField = new Field2d();
    private static GcStatsCollector gscollect = new GcStatsCollector();
    public static Alert gcAlert = new Alert("MEMORY TWEAKING LOOK INTO TS RN", AlertType.kWarning);

    static NetworkTableEntry turretSetpoint;
    EpilogueBackend backend;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    public Robot() {
        Epilogue.configure(config -> {
            config.root = "JohnLogs";
            config.loggingPeriod = Seconds.of(0.02);
            config.backend = new NTEpilogueBackend(NetworkTableInstance.getDefault());
            config.minimumImportance = Logged.Importance.DEBUG;
        });
        //Epilogue.bind(this);
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
        teleopTab.add("Turret Position",
                0)
                .withProperties(Map.of("min_value", -0.5, "max_value", 0.5, "block increment", 0.1, "Divisions", 6));
        turretSetpoint = NetworkTableInstance.getDefault().getTable("Shuffleboard")
                .getSubTable(teleopTab.getTitle())
                .getEntry("Turret Position");

        ;

        // disabledTab.add()

    }
    void updateEpilogue(){
        var cfg = edu.wpi.first.epilogue.Epilogue.getConfig();
        if (cfg.backend != null && cfg.root != null) {
            // base: JohnLogs
            var rootBackend = cfg.backend.getNested(cfg.root);

            edu.wpi.first.epilogue.Epilogue.robotLogger.tryUpdate(
                rootBackend.getNested("Robot"),
                this,
                cfg.errorHandler);
            // call the generated turret logger with that nested backend
            edu.wpi.first.epilogue.Epilogue.turretLogger.tryUpdate(
                rootBackend.getNested("Turret"),
                frc.robot.subsystems.Turret.Turret.getInstance(),
                cfg.errorHandler);
            edu.wpi.first.epilogue.Epilogue.turretShooterLogger.tryUpdate(
                rootBackend.getNested("Turret Shooter"),
                frc.robot.subsystems.Turret.TurretShooter.getInstance(),
                cfg.errorHandler);
            edu.wpi.first.epilogue.Epilogue.kirbyLogger.tryUpdate(
                rootBackend.getNested("Kirby"),
                frc.robot.subsystems.Kirby.getInstance(),
                cfg.errorHandler);
        }
    }
    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
        updateEpilogue();
        gscollect.update();
        teleopField.setRobotPose(drivetrain.getPose());
        if (Robot.isSimulation()) {
            NetworkTableInstance.getDefault().flush();
        }

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
        Turret.getInstance().setIronmansetpoint(turretSetpoint.getDouble(0));

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