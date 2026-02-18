// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.nio.file.Paths;
import java.util.List;
import com.ctre.phoenix6.HootAutoReplay;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ElasticConstants.whoCanScore;
import frc.robot.Stable.TimedRobotstangs;
import frc.robot.commands.IntakeCommands.ExtendIntake;
import frc.robot.commands.IntakeCommands.RetractIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret.Hood;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.Flywheel;
@Logged
public class Robot extends TimedRobotstangs {
    public static boolean isJaba;
    CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    private final RobotContainer m_robotContainer;
    public static ShuffleboardTab disableTab, teleopTab;

    public static Field2d teleopField = new Field2d();
    private static GcStatsCollector gscollect = new GcStatsCollector();
    private int lastMatchSecond = -1;

    public static Alert gcAlert = new Alert("MEMORY TWEAKING LOOK INTO TS RN", AlertType.kError);
    public static Alert shiftAlert = new Alert("Alliance shift may not be accurate", AlertType.kWarning);

    public static Constants.ElasticConstants.whoCanScore canBlueScore;

    public Color allianceScore = Constants.ElasticConstants.purple;

    static NetworkTableEntry turretSetpoint;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        Epilogue.configure(config -> {
            config.root = "JohnLogs";
            config.loggingPeriod = Seconds.of(0.02);
            config.backend = new NTEpilogueBackend(NetworkTableInstance.getDefault());
            config.minimumImportance = Logged.Importance.DEBUG;
        });
    }

    void updateEpilogue() {
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
            edu.wpi.first.epilogue.Epilogue.climberLogger.tryUpdate(
                    rootBackend.getNested("Climber"),
                    frc.robot.subsystems.Climber.getInstance(),
                    cfg.errorHandler);
            edu.wpi.first.epilogue.Epilogue.hoodLogger.tryUpdate(
                    rootBackend.getNested("Hood"),
                    frc.robot.subsystems.Turret.Hood.getInstance(),
                    cfg.errorHandler);
            edu.wpi.first.epilogue.Epilogue.feederLogger.tryUpdate(
                    rootBackend.getNested("Feeder"),
                    frc.robot.subsystems.Feeder.getInstance(),
                    cfg.errorHandler);
            edu.wpi.first.epilogue.Epilogue.kirbyLogger.tryUpdate(
                    rootBackend.getNested("Kirby"),
                    frc.robot.subsystems.Kirby.getInstance(),
                    cfg.errorHandler);
            edu.wpi.first.epilogue.Epilogue.flywheelLogger.tryUpdate(
                    rootBackend.getNested("Flywheel"),
                    frc.robot.subsystems.Turret.Flywheel.getInstance(),
                    cfg.errorHandler);
        }
    }

    @Override
    public void robotInit() {
        isJaba = Paths.get("/home/lvuser/Ultron").toFile().exists();

        SmartDashboard.putData("Field", teleopField);

        configureDisabledTab();
        configureTeleopTab();
        loadNamedCommands();




    }


    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
        updateEpilogue();
        gscollect.update();
        SmartDashboard.putString("Alliance can score", allianceScore.toHexString());
        
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

    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        lastMatchSecond = (int) DriverStation.getMatchTime();

    }

    @Override
    public void teleopPeriodic() {
        shiftAlert.set(canBlueScore == whoCanScore.error);
        String gameData = DriverStation.getGameSpecificMessage();
        int gameTimeInt = (int) DriverStation.getMatchTime();
        double gameTime = DriverStation.getMatchTime();

        if (gameTime < 30 || gameTime > 130) {
            canBlueScore = whoCanScore.both;
        } else if (gameTime < 130 && gameTime > 129) {
            if (gameData.length() > 0) {
                switch (gameData.charAt(0)) {
                    case 'B':
                        canBlueScore = whoCanScore.red;
                        break;
                    case 'R':
                        canBlueScore = whoCanScore.blue;
                        break;
                    default:
                        canBlueScore = whoCanScore.error;
                        break;
                }
            } else {
                canBlueScore = whoCanScore.error;

            }
        }

        else if ((gameTimeInt - 10) % 25 == 0) {
            canBlueScore = (canBlueScore == whoCanScore.blue) ? whoCanScore.red : whoCanScore.blue;
        } else {
            int secondsSince10 = gameTimeInt - 10;
            if (secondsSince10 >= 0 && secondsSince10 % 25 == 0) {
                if (gameTimeInt != lastMatchSecond) {
                    canBlueScore = (canBlueScore == whoCanScore.blue) ? whoCanScore.red : whoCanScore.blue;
                }
            }
        }

        lastMatchSecond = gameTimeInt;

        if (canBlueScore == whoCanScore.blue) {
            allianceScore = Constants.ElasticConstants.blue;
        } else if (canBlueScore == whoCanScore.red) {
            allianceScore = Constants.ElasticConstants.red;
        } else if (canBlueScore == whoCanScore.both) {
            allianceScore = Constants.ElasticConstants.purple;
        } else {
            allianceScore = Constants.ElasticConstants.white;
        }

        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
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

        public void configureDisabledTab(){

        disableTab = Shuffleboard.getTab("Disabled");

        disableTab.add("Set Drivetrain motors to coast",
                new InstantCommand(() -> CommandSwerveDrivetrain.getInstance().setCoastMode(true))
                        .ignoringDisable(true));
        disableTab.add("Set Drivetrain motors to brake",
                new InstantCommand(() -> CommandSwerveDrivetrain.getInstance().setCoastMode(false))
                        .ignoringDisable(true));

        disableTab.add("RESET TO SUN TZUUUUUU",
                new InstantCommand(() -> Turret.getInstance().resetPositionWithSunTzu())
                        .ignoringDisable(true));
    }

    public void configureTeleopTab(){
        teleopTab = Shuffleboard.getTab("Teleop");


        
    }

public void loadNamedCommands(){

        NamedCommands.registerCommand("Shoot", new InstantCommand(() -> Turret.getInstance().trackHub()));

        NamedCommands.registerCommand("Extend", new ExtendIntake());

        NamedCommands.registerCommand("Retract", new RetractIntake());

        NamedCommands.registerCommand("Dont shoot",
                new InstantCommand(() -> Flywheel.getInstance().setShouldShoot(false))
                        .finallyDo(() -> Flywheel.getInstance().setShouldShoot(true)));

        NamedCommands.registerCommand("Hood down",
                new InstantCommand(() -> Hood.getInstance().setHoodPosition(Constants.HoodConstants.kHoodDownPos))
                        .alongWith(new InstantCommand(() -> Flywheel.getInstance().setShouldShoot(false)))
                        .finallyDo(() -> Flywheel.getInstance().setShouldShoot(true)));

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