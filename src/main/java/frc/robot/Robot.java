// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElasticConstants.whoCanScore;
import frc.robot.Stable.TimedRobotstangs;
import frc.robot.commands.Index;
import frc.robot.commands.IntakeCommands.ExtendIntake;
import frc.robot.commands.IntakeCommands.RetractIntake;
import frc.robot.commands.TurretCommands.Flywheel.ChargeUp;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
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
    public static Alert autoAlert = new Alert("Auto couldnt load", AlertType.kError);

    public static Constants.ElasticConstants.whoCanScore canBlueScore;

    public Color allianceScore = Constants.ElasticConstants.purple;

    private SendableChooser<Command> autoChooser = new SendableChooser<>();
    private SendableChooser<Command> autoChooserFav = new SendableChooser<>();

    Command autoCommand;

    static NetworkTableEntry turretSetpoint;

 

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
        CommandScheduler.getInstance().run();
        updateEpilogue();
        gscollect.update();
        SmartDashboard.putString("Alliance can score", allianceScore.toHexString());

    }

    @Override
    public void disabledInit() {
        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooserFav = AutoBuilder.buildAutoChooserWithOptionsModifier(
        (stream) -> stream.filter(auto -> auto.getName().startsWith("fav")));
        
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("Favorite Auto Chooser", autoChooserFav);
    }

    @Override
    public void disabledPeriodic() {

        
        if( autoChooser.getSelected() != null &&
            !autoChooser.getSelected().getName().equals("none")){
            autoCommand = autoChooser.getSelected();
        } else {
            autoCommand = autoChooserFav.getSelected();
        }
        drawCurrentAuto(autoCommand.getName());
    }

    @Override
    public void disabledExit() {
        drawCurrentAuto(null);
    }

    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().schedule(autoCommand);

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

    public void configureDisabledTab() {

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

    public void logCurrentAuto() {

    }

    public void configureTeleopTab() {
        teleopTab = Shuffleboard.getTab("Teleop");

    }

    public void loadNamedCommands() {

        NamedCommands.registerCommand("Shoot",
                new InstantCommand(() -> Turret.getInstance().trackHub()).withTimeout(0.5));

        NamedCommands.registerCommand("Extend", new ExtendIntake().withTimeout(0.5));

        NamedCommands.registerCommand("Retract", new RetractIntake().withTimeout(0.5));

        NamedCommands.registerCommand("Dont shoot",
                new InstantCommand(() -> Flywheel.getInstance().setShouldShoot(false))
                        .finallyDo(() -> Flywheel.getInstance().setShouldShoot(true)));

        NamedCommands.registerCommand("Hood down",
                new InstantCommand(() -> Hood.getInstance().setHoodPosition(Constants.HoodConstants.kHoodDownPos))
                        .alongWith(new InstantCommand(() -> Flywheel.getInstance().setShouldShoot(false)))
                        .finallyDo(() -> Flywheel.getInstance().setShouldShoot(true)));
        NamedCommands.registerCommand("Photoshoot", 
                (new ChargeUp(Constants.FlywheelConstants.kShootSpeed).alongWith(new Index()))
                .raceWith(new WaitUntilCommand(() -> Feeder.getInstance().getfuelSensor()))
        );

    }

    public void drawCurrentAuto(String autoName) {
        List<Pose2d> poses = new ArrayList<>();
        poses.clear();
        if(autoName != null){
        try {
            for (PathPlannerPath path : PathPlannerAuto.getPathGroupFromAutoFile(autoName)) {
                for (Pose2d pose : path.getPathPoses()) {
                    poses.add(pose);
                }

            }
            autoAlert.set(false);
        } catch (Exception e) {
            e.printStackTrace();
            autoAlert.set(true);
        }}
        
        teleopField.getObject("auto").setPoses(poses);
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