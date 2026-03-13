// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.subsystems.Intake.*;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
// import edu.wpi.first.epilogue.Epilogue;
// import edu.wpi.first.epilogue.Logged;
// import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElasticConstants.whoCanScore;
import frc.robot.Stable.TimedRobotstangs;
import frc.robot.commands.Index;
import frc.robot.commands.HopperCommands.RunHopper;
import frc.robot.commands.IntakeCommands.ExtendIntake;
import frc.robot.commands.IntakeCommands.RetractIntake;
import frc.robot.commands.IntakeCommands.StartIntake;
import frc.robot.commands.TurretCommands.Hood.HomeHood;
import frc.robot.commands.TurretCommands.Hood.setHoodPosition;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.commands.TurretCommands.TurretFactory;
import frc.robot.commands.TurretCommands.Flywheel.ChargeUp;
import frc.robot.subsystems.Turret.Hood;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.Flywheel;

// @Logged
public class Robot extends TimedRobotstangs {
    public static boolean isJaba;
    CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    public static RobotContainer m_robotContainer;
    public static ShuffleboardTab disableTab, teleopTab;

    public static Field2d teleopField = new Field2d();
    private static GcStatsCollector gscollect = new GcStatsCollector();
    private int lastMatchSecond = -1;

    public static Alert gcAlert = new Alert("MEMORY TWEAKING LOOK INTO TS RN", AlertType.kError);
    public static Alert shiftAlert = new Alert("Alliance shift may not be accurate", AlertType.kWarning);
    public static Alert autoAlert = new Alert("Auto couldnt load", AlertType.kError);

    public static Constants.ElasticConstants.whoCanScore canBlueScore;

    String autoname = "";
    String oldAutoname = "";

    public CommandScheduler scheduler = CommandScheduler.getInstance();

    public Color allianceScore = Constants.ElasticConstants.purple;

    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    public PowerDistribution pdh = new PowerDistribution(Constants.ElasticConstants.kPdhCanId, ModuleType.kRev);

    // pit test choosers
    SendableChooser<Command> hoodSetpointChooser = new SendableChooser<>();
    SendableChooser<Command> intakePivotSetpointChooser = new SendableChooser<>();
    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();
    Command autoCommand, autoGroup;

    static NetworkTableEntry turretSetpoint;

    public Robot() {
        m_robotContainer = new RobotContainer();
        // Epilogue.configure(config -> {
        //     config.root = "JohnLogs";
        //     config.loggingPeriod = Seconds.of(0.02);
        //     config.backend = new NTEpilogueBackend(NetworkTableInstance.getDefault());
        //     config.minimumImportance = Logged.Importance.DEBUG;
        // });
    }

    void postPitTestStatus(String status) {
        SmartDashboard.putString("PitTest/Pit Test Status", status);
    }

    void updateEpilogue() {
        // var cfg = edu.wpi.first.epilogue.Epilogue.getConfig();
        // if (cfg.backend != null && cfg.root != null) {
        //     // base: JohnLogs
        //     var rootBackend = cfg.backend.getNested(cfg.root);

        //     edu.wpi.first.epilogue.Epilogue.robotLogger.tryUpdate(
        //             rootBackend.getNested("Robot"),
        //             this,
        //             cfg.errorHandler);
        //     // call the generated turret logger with that nested backend
        //     edu.wpi.first.epilogue.Epilogue.turretLogger.tryUpdate(
        //             rootBackend.getNested("Turret"),
        //             frc.robot.subsystems.Turret.Turret.getInstance(),
        //             cfg.errorHandler);
        //     edu.wpi.first.epilogue.Epilogue.climberLogger.tryUpdate(
        //             rootBackend.getNested("Climber"),
        //             frc.robot.subsystems.Climber.getInstance(),
        //             cfg.errorHandler);
        //     edu.wpi.first.epilogue.Epilogue.hoodLogger.tryUpdate(
        //             rootBackend.getNested("Hood"),
        //             frc.robot.subsystems.Turret.Hood.getInstance(),
        //             cfg.errorHandler);
        //     edu.wpi.first.epilogue.Epilogue.feederLogger.tryUpdate(
        //             rootBackend.getNested("Feeder"),
        //             frc.robot.subsystems.Feeder.getInstance(),
        //             cfg.errorHandler);
        //     edu.wpi.first.epilogue.Epilogue.kirbyLogger.tryUpdate(
        //             rootBackend.getNested("Kirby"),
        //             frc.robot.subsystems.Intake.Kirby.getInstance(),
        //             cfg.errorHandler);
        //     edu.wpi.first.epilogue.Epilogue.flywheelLogger.tryUpdate(
        //             rootBackend.getNested("Flywheel"),
        //             frc.robot.subsystems.Turret.Flywheel.getInstance(),
        //             cfg.errorHandler);
        // }
    }

    @Override
    public void robotInit() {

        isJaba = true;
        SmartDashboard.putData("Field", teleopField);

        DataLogManager.start("");
        DataLogManager.log("Driverstation connected");
        DriverStation.startDataLog(DataLogManager.getLog());

        CommandScheduler.getInstance()
                .onCommandInitialize((action) -> DataLogManager.log(action.getName() + " Command Initialized"));
        CommandScheduler.getInstance()
                .onCommandInterrupt((action, cause) -> {
                    String reason = cause.map(c -> c.getName()).orElse("no cause");
                    DataLogManager.log(action.getName() + " Command Interrupted by: " + reason);
                    SmartDashboard.putString("Auto/LastInterrupt", action.getName() + " <- " + reason);
                });
        CommandScheduler.getInstance()
                .onCommandFinish((action) -> DataLogManager.log(action.getName() + " Command Finished"));

        configureDisabledTab();
        configureTeleopTab();
        loadNamedCommands();

    }

    void setupPitTest() {
        CommandSwerveDrivetrain.getInstance().runOnce(() -> CommandSwerveDrivetrain.getInstance().seedFieldCentric());
        postPitTestStatus("Pit Test Ready");
        SmartDashboard.putNumber("PitTest/HoodDC", 0);
        SmartDashboard.putNumber("PitTest/FlywheelDC", 0);
        SmartDashboard.putNumber("PitTest/FlywheelVelocity", 0);
        SmartDashboard.putNumber("PitTest/TurretPosition", 0);
        SmartDashboard.putNumber("PitTest/TurretDC", 0);
        SmartDashboard.putNumber("PitTest/IntakeWheelsDC", 0);
        SmartDashboard.putNumber("PitTest/FeederDC", 0);
        SmartDashboard.putNumber("PitTest/IntakePivotDC", 0);
        hoodSetpointChooser.addOption("Down",
                Hood.getInstance().runOnce(() -> {
                    Hood.getInstance().setHoodPosition(Constants.HoodConstants.kHoodDownPos);
                }));
        hoodSetpointChooser.addOption("Up",
                Hood.getInstance().runOnce(() -> {
                    Hood.getInstance().setHoodPosition(Constants.HoodConstants.kHoodUpPos);
                }));
        SmartDashboard.putData("PitTest/HoodSetpoints", hoodSetpointChooser);

        intakePivotSetpointChooser.addOption("Extend",
                new ExtendIntake(false));
        intakePivotSetpointChooser.addOption("Retract",
                new RetractIntake());
        SmartDashboard.putData("PitTest/IntakePivotSetpoints", intakePivotSetpointChooser);

        SwerveRequest forwardRequest = new SwerveRequest.FieldCentric().withVelocityX(-0.25);
        Command driveForwardCommand = CommandSwerveDrivetrain.getInstance().applyRequest(() -> forwardRequest);
        SwerveRequest backRequest = new SwerveRequest.FieldCentric().withVelocityX(0.25);
        Command driveBackCommand = CommandSwerveDrivetrain.getInstance().applyRequest(() -> backRequest);
        SwerveRequest leftRequest = new SwerveRequest.FieldCentric().withVelocityY(-0.25);
        Command driveLeftCommand = CommandSwerveDrivetrain.getInstance().applyRequest(() -> leftRequest);
        SwerveRequest rightRequest = new SwerveRequest.FieldCentric().withVelocityY(0.25);
        Command driveRightCommand = CommandSwerveDrivetrain.getInstance().applyRequest(() -> rightRequest);

        // actual pit test automated
        Command pitTestCommand = CommandSwerveDrivetrain.getInstance()
                .runOnce(() -> CommandSwerveDrivetrain.getInstance().seedFieldCentric())
                .andThen(new InstantCommand(() -> postPitTestStatus("Extending Intake")))
                .andThen(new ExtendIntake(false))
                .andThen(new InstantCommand(() -> postPitTestStatus("Starting Intake Wheels")))
                .andThen(new StartIntake().withTimeout(7))
                .andThen(new InstantCommand(() -> postPitTestStatus("Running Hopper")))
                .andThen(new RunHopper().withTimeout(7))
                .andThen(new InstantCommand(() -> postPitTestStatus("Charging up Flywheel")))
                .andThen(new ChargeUp(false).withTimeout(7))
                .andThen(new InstantCommand(() -> postPitTestStatus("Starting Feeder")))
                .andThen(new Index(false).withTimeout(7))
                .andThen(new InstantCommand(() -> postPitTestStatus("Testing Hood")))
                .andThen(new setHoodPosition(false).withTimeout(7))
                .andThen(new InstantCommand(() -> postPitTestStatus("Retracting Intake")))
                .andThen(new RetractIntake())
                .andThen(new InstantCommand(() -> postPitTestStatus("Driving Forward")))
                .andThen(() -> driveForwardCommand.withTimeout(7))
                .andThen(new InstantCommand(() -> postPitTestStatus("Driving Left")))
                .andThen(() -> driveLeftCommand.withTimeout(7))
                .andThen(new InstantCommand(() -> postPitTestStatus("Finished Pit Test")));

        SmartDashboard.putData("Run Auto pit test (Intake must be up)", pitTestCommand);

        SmartDashboard.putData("PitTest/Drive Forward", driveForwardCommand);
        SmartDashboard.putData("PitTest/Drive Left", driveLeftCommand);
        SmartDashboard.putData("PitTest/Drive Back", driveBackCommand);
        SmartDashboard.putData("PitTest/Drive Right", driveRightCommand);
    }

    void pitTestPeriodic() {

        // read sliders and run commands
        double hoodDC = SmartDashboard.getNumber("PitTest/HoodDC", 0);
        double flywheelDC = SmartDashboard.getNumber("PitTest/FlywheelDC", 0);
        double flywheelVelocity = SmartDashboard.getNumber("PitTest/FlywheelVelocity", 0);
        double turretPosition = SmartDashboard.getNumber("PitTest/TurretPosition", 0);
        double intakeWheelsDC = SmartDashboard.getNumber("PitTest/IntakeWheelsDC", 0);
        double feederDC = SmartDashboard.getNumber("PitTest/FeederDC", 0);
        double intakePivotDC = SmartDashboard.getNumber("PitTest/IntakePivotDC", 0);
        double turretDC = SmartDashboard.getNumber("PitTest/TurretDC", 0);

        if (hoodDC != 0) {
            Hood.getInstance().setHoodDutyCycle(hoodDC);
        }
        if (intakeWheelsDC != 0) {
            Kirby.getInstance().setKirbyWheels(intakeWheelsDC);
        }
        if (turretDC != 0) {
            Turret.getInstance().runTurretDutyCycle(turretDC);
        }
        if (feederDC != 0) {
            Feeder.getInstance().setDutyCycle(feederDC);
        }
        if (intakePivotDC != 0) {
            IntakePivot.getInstance().setDC(intakePivotDC);
        }
        if (turretPosition != 0) {
            Turret.setManualTarget(true);
            Turret.getInstance().setTurretSetpointDoubleSupplier(() -> turretPosition);
        }
        if (flywheelDC != 0) {
            Flywheel.getInstance().setDutyCycle(flywheelDC);
        }
        if (flywheelVelocity != 0) {
            Flywheel.getInstance().setShooterRPS(flywheelVelocity);
        }

    }

    @Override
    public void robotPeriodic() {

        CommandScheduler.getInstance().run();
        gscollect.update();
        SmartDashboard.putString("Alliance can score", allianceScore.toHexString());

    }

    @Override
    public void disabledInit() {

        autoChooser = AutoBuilder.buildAutoChooser();
        // autoChooserFav = AutoBuilder.buildAutoChooserWithOptionsModifier(
        // (stream) -> stream.filter(auto -> auto.getName().startsWith("fav")));

        SmartDashboard.putData("Auto Chooser", autoChooser);

        autoname = autoChooser.getSelected().getName();
    }

    @Override
    public void disabledPeriodic() {

        autoname = autoChooser.getSelected().getName();

        autoCommand = autoChooser.getSelected();

        SmartDashboard.putString("Auto/Auto Name", autoCommand.getName());

        // if (!oldAutoname.equals(autoname)) {
            // drawCurrentAuto(autoCommand.getName());
        // }

        oldAutoname = autoChooser.getSelected().getName();
    }

    @Override
    public void disabledExit() {
        // drawCurrentAuto(null);

        autoGroup = new SequentialCommandGroup(
                // new InstantCommand(() -> Turret.getInstance().resetPositionWithSunTzu()),
                new InstantCommand(() -> Hood.getInstance().setHoodPositionAuto(0)),
                new InstantCommand(() -> IntakePivot.getInstance().setPosition(Constants.KirbyConstants.kHomePosition)),
                new InstantCommand(() -> Turret.getInstance().zeroTurret()),

                autoCommand);

    }


    //TODO driver cam
    @Override
    public void autonomousInit() {

        CommandScheduler.getInstance().cancelAll();

        CommandScheduler.getInstance().schedule(
                autoGroup);
    }

    @Override
    public void autonomousPeriodic() {
        SmartDashboard.putBoolean("Auto/CommandRunning",
                autoCommand != null && CommandScheduler.getInstance().isScheduled(autoCommand));
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        lastMatchSecond = Robot.isSimulation() ? (int) Timer.getFPGATimestamp() : (int) DriverStation.getMatchTime();

    }

    @Override
    public void teleopPeriodic() {
        shiftAlert.set(canBlueScore == whoCanScore.error);
        String gameData = DriverStation.getGameSpecificMessage();
        int gameTimeInt = (int) DriverStation.getMatchTime();
        double gameTime = DriverStation.getMatchTime();
        if (Robot.isSimulation()) {
            gameTimeInt = (int) (Timer.getFPGATimestamp() - lastMatchSecond);
            gameTime = Timer.getFPGATimestamp() - lastMatchSecond;
        } else {
            gameTimeInt = (int) DriverStation.getMatchTime();
            gameTime = DriverStation.getMatchTime();
        }

        if (gameTime < 30 || gameTime > 130) {
            if (gameTimeInt == 29) {
                CommandScheduler.getInstance().schedule(RobotContainer.rumbleDriveController(2));
                CommandScheduler.getInstance().schedule(RobotContainer.rumbleManipController(2));
            }
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
            CommandScheduler.getInstance().schedule(RobotContainer.rumbleDriveController(1));
            CommandScheduler.getInstance().schedule(RobotContainer.rumbleManipController(1));
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

        SmartDashboard.putNumber("Match Time", gameTime);
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        setupPitTest();
        SmartDashboard.putNumber("Hood setpoint", 0);
        SmartDashboard.putNumber("Flywheel velocity", 0);
        SwerveRequest rightRequest = new SwerveRequest.RobotCentric().withVelocityY(-0.5);
        Command driveRightCommand = CommandSwerveDrivetrain.getInstance().applyRequest(() -> rightRequest);

        SmartDashboard.putData("Do the wheel john", driveRightCommand);

    }

    @Override
    public void testPeriodic() {
        // pitTestPeriodic();

    }

    @Override
    public void testExit() {
    }

    public static boolean isRed() {

      
        // if(!DriverStation.isDSAttached()){
        //     return false;
        // }
        try{
             if (DriverStation.getAlliance().get() == null) {
            return false;
        }
        if (DriverStation.getAlliance().get() == (Alliance.Red)) {
            return true;
        } else {
            return false;
        }}
        catch(Exception e){
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

        disableTab.add("Zero Hood",
                new InstantCommand(() -> Hood.getInstance().setHoodPosition(0)).ignoringDisable(true));
 

        disableTab.add("RESET TO SUN TZUUUUUU",
                new InstantCommand(() -> Turret.getInstance().resetPositionWithSunTzu())
                        .ignoringDisable(true));


        disableTab.add("ZeroTurret",
        new InstantCommand(() -> Turret.getInstance().zeroTurret())
                                .ignoringDisable(true));

    
    
    }

    public void logCurrentAuto() {

    }

    public void configureTeleopTab() {
        teleopTab = Shuffleboard.getTab("Teleop");

    }

    public void loadNamedCommands() {

        NamedCommands.registerCommand("Shoot",
                new PrintCommand("your supposed to shoot bruh"));
        NamedCommands.registerCommand("Pass",
                new PrintCommand("your supposed to pass bruh"));

        NamedCommands.registerCommand("Extend",new ExtendIntake(false));

        // new ExtendIntake(false).withTimeout(0.5));

        NamedCommands.registerCommand("Retract", new RetractIntake().withTimeout(0.5));

        NamedCommands.registerCommand("Dontshoot",
                new InstantCommand(() -> Flywheel.getInstance().setShouldShoot(false))
                        .finallyDo(() -> Flywheel.getInstance().setShouldShoot(true)));

        NamedCommands.registerCommand("Hooddown",
                new InstantCommand(() -> Hood.getInstance().setHoodPosition(Constants.HoodConstants.kHoodDownPos))
                        .alongWith(new InstantCommand(() -> Flywheel.getInstance().setShouldShoot(false)))
                        .finallyDo(() -> Flywheel.getInstance().setShouldShoot(true)));

        NamedCommands.registerCommand("Photoshoot",
                (TurretFactory.Shoot().alongWith(new RunHopper()).alongWith(new StartIntake()))
                // .until(() -> Feeder.getInstance().getfuelSensor())
                );

        NamedCommands.registerCommand("BumpIntake", new ExtendIntake(true));

        NamedCommands.registerCommand("Climb",
                new PrintCommand("Hi!"));

    }

    public void drawCurrentAuto(String autoName) {
        List<Pose2d> poses = new ArrayList<>();
        poses.clear();
        if (autoName != null && !autoName.equals("")) {
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
            }
            teleopField.getObject("auto").setPoses(poses);

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