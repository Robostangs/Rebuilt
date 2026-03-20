// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.subsystems.Intake.*;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
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
import frc.robot.Constants.ElasticConstants.whoCanScore;
import frc.robot.Stable.LimelightHelpers;
import frc.robot.Stable.TimedRobotstangs;
import frc.robot.commands.Index;
import frc.robot.commands.DumperCommands.dumpFactory;
import frc.robot.commands.DumperCommands.Flywheel.ChargeUp;
import frc.robot.commands.DumperCommands.Hood.setHoodPosition;
import frc.robot.commands.HopperCommands.RunHopper;
import frc.robot.commands.IntakeCommands.ExtendIntake;
import frc.robot.commands.IntakeCommands.RetractIntake;
import frc.robot.commands.IntakeCommands.StartIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Dumper.Flywheel;
import frc.robot.subsystems.Dumper.Hood;


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

    private SendableChooser<String> autoChooser = new SendableChooser<>();

    public PowerDistribution pdh = new PowerDistribution(Constants.ElasticConstants.kPdhCanId, ModuleType.kRev);

    public static boolean shooterPass=false;
    public static boolean shooterIsDumb=false;
    // pit test choosers
    SendableChooser<Command> hoodSetpointChooser = new SendableChooser<>();
    SendableChooser<Command> intakePivotSetpointChooser = new SendableChooser<>();
    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();
    Command autoCommand, autoGroup;

    static NetworkTableEntry turretSetpoint;
    static boolean isPassing = false;
    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    void postPitTestStatus(String status) {
        SmartDashboard.putString("PitTest/Pit Test Status", status);
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
                new ExtendIntake());
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
                .andThen(new ExtendIntake())
                .andThen(new InstantCommand(() -> postPitTestStatus("Starting Intake Wheels")))
                .andThen(new StartIntake().withTimeout(7))
                .andThen(new InstantCommand(() -> postPitTestStatus("Running Hopper")))
                .andThen(new RunHopper().withTimeout(7))
                .andThen(new InstantCommand(() -> postPitTestStatus("Charging up Flywheel")))
                .andThen(new ChargeUp().withTimeout(7))
                .andThen(new InstantCommand(() -> postPitTestStatus("Starting Feeder")))
                .andThen(new Index(false).withTimeout(7))
                .andThen(new InstantCommand(() -> postPitTestStatus("Testing Hood")))
                .andThen(new setHoodPosition().withTimeout(7))
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

        //TODO make PIT test as a whole look better 
        // read sliders and run commands
        double hoodDC = SmartDashboard.getNumber("PitTest/HoodDC", 0);
        double flywheelDC = SmartDashboard.getNumber("PitTest/FlywheelDC", 0);
        double flywheelVelocity = SmartDashboard.getNumber("PitTest/FlywheelVelocity", 0);
        double intakeWheelsDC = SmartDashboard.getNumber("PitTest/IntakeWheelsDC", 0);
        double feederDC = SmartDashboard.getNumber("PitTest/FeederDC", 0);
        double intakePivotDC = SmartDashboard.getNumber("PitTest/IntakePivotDC", 0);

        if (hoodDC != 0) {
            Hood.getInstance().setHoodDutyCycle(hoodDC);
        }
        if (intakeWheelsDC != 0) {
            Kirby.getInstance().setKirbyWheels(intakeWheelsDC);
        }
    
        if (feederDC != 0) {
            Feeder.getInstance().setDutyCycle(feederDC);
        }
        if (intakePivotDC != 0) {
            IntakePivot.getInstance().setDC(intakePivotDC);
        }
        if (flywheelDC != 0) {
            Flywheel.getInstance().setFlywheelDC(flywheelDC);
        }
        if (flywheelVelocity != 0) {
            Flywheel.getInstance().setFlywheelVelocity(flywheelVelocity);
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

        autoChooser.addOption("FavNormal", "favCenter - Shoot - DepotIntake - Shoot");
        autoChooser.addOption("Just pre loads", "shoot");

        SmartDashboard.putData("Auto Chooser", autoChooser);

        autoname = autoChooser.getSelected();
    }

    @Override
    public void disabledPeriodic() {

        autoname = autoChooser.getSelected();

        if (autoname != null) {
            if (autoname.equals("favCenter - Shoot - DepotIntake - Shoot")) {
                autoCommand = new PathPlannerAuto(autoname);
            } else if (autoname.equals("shoot")) {
                autoCommand = (dumpFactory.Shoot().alongWith(new RunHopper()).alongWith(new StartIntake()));

            } else {
                autoCommand = new PrintCommand("Auto not Autoing....");

            }
        } else {
            autoCommand = new PrintCommand("Auto not Autoing....");

        }
        SmartDashboard.putString("Auto/Auto Name", autoCommand.getName());

        // if (!oldAutoname.equals(autoname)) {
        // drawCurrentAuto(autoCommand.getName());
        // }

        oldAutoname = " autoChooser.getSelected().getName();";
    }

    @Override
    public void disabledExit() {
        // drawCurrentAuto(null);

        autoGroup = new SequentialCommandGroup(
                new InstantCommand(() -> Hood.getInstance().setHoodPositionAuto(0)),
                new InstantCommand(() -> IntakePivot.getInstance().setPosition(Constants.KirbyConstants.kHomePosition)),

                autoCommand);

    }

    @Override
    public void autonomousInit() {

        LimelightHelpers.setRewindEnabled(Constants.VisionConstants.kLimelightFour, true);
                LimelightHelpers.setRewindEnabled(Constants.VisionConstants.kLimelightFourR, true);

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
        double shiftTimeLeft = 0;

        // endgame
        if (gameTime <= 30.0) {
            shiftTimeLeft = gameTimeInt;
            canBlueScore = whoCanScore.both;

            // Rumble only once on entry (when it first hits 30)
            if (gameTime>29 && gameTime<30) {
                CommandScheduler.getInstance().schedule(RobotContainer.rumbleDriveController(40));
                CommandScheduler.getInstance().schedule(RobotContainer.rumbleManipController(40));
            }

        // transition 10 seconds
        } else if (gameTime > 130.0) {
            canBlueScore = whoCanScore.both;
            shiftTimeLeft = gameTime - 130; 
            if(shiftTimeLeft>4 && shiftTimeLeft<5){
                CommandScheduler.getInstance().schedule(RobotContainer.rumbleDriveController(1));
                CommandScheduler.getInstance().schedule(RobotContainer.rumbleManipController(1));
            }
        

        // shifts

        } else {
            if(gameTime>128){
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
                    // this is js a placeholder
                    canBlueScore = whoCanScore.blue;
                }
            }

            double secondsIntoShifts = 130 - gameTime; 
            double positionInShift = secondsIntoShifts % 25; 
            shiftTimeLeft = 25 - positionInShift;    

            // change of shift
            if (shiftTimeLeft <0.1) {
                canBlueScore = (canBlueScore == whoCanScore.blue) ? whoCanScore.red : whoCanScore.blue;
            }
            if(shiftTimeLeft<5&&shiftTimeLeft>4.5){
                CommandScheduler.getInstance().schedule(RobotContainer.rumbleDriveController(1));
                CommandScheduler.getInstance().schedule(RobotContainer.rumbleManipController(1));
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
        SmartDashboard.putNumber("Shift Time Left", shiftTimeLeft);
    }
    @Override
    public void teleopExit() {

        if (DriverStation.isFMSAttached()) {
            LimelightHelpers.triggerRewindCapture(Constants.VisionConstants.kLimelightFour, 150);
            LimelightHelpers.triggerRewindCapture(Constants.VisionConstants.kLimelightFourR, 150);
        }
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
        pitTestPeriodic();

    }

    @Override
    public void testExit() {
    }

    public static boolean isRed() {

        // if(!DriverStation.isDSAttached()){
        // return false;
        // }
        try {
            if (DriverStation.getAlliance().get() == null) {
                return false;
            }
            if (DriverStation.getAlliance().get() == (Alliance.Red)) {
                return true;
            } else {
                return false;
            }
        } catch (Exception e) {
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


    }

    public void logCurrentAuto() {

    }

    public void configureTeleopTab() {
        teleopTab = Shuffleboard.getTab("Teleop");

    }

    public void loadNamedCommands() {

        NamedCommands.registerCommand("Shoot",
        //devin learn apostrophes bro this was previously "your"
                new PrintCommand("you're supposed to shoot bruh"));
        NamedCommands.registerCommand("Pass",
                new PrintCommand("you're supposed to pass bruh"));

        NamedCommands.registerCommand("Extend", new ExtendIntake());

        NamedCommands.registerCommand("Retract", new RetractIntake().withTimeout(0.5));

        NamedCommands.registerCommand("Dontshoot",
                new InstantCommand(() -> Flywheel.getInstance().setCanShoot(false))
                        .finallyDo(() -> Flywheel.getInstance().setCanShoot(true)));

        NamedCommands.registerCommand("Hooddown",
                new InstantCommand(() -> Hood.getInstance().setHoodPosition(Constants.HoodConstants.kHoodDownPos))
                        .alongWith(new InstantCommand(() -> Flywheel.getInstance().setCanShoot(false)))
                        .finallyDo(() -> Flywheel.getInstance().setCanShoot(true)));

        NamedCommands.registerCommand("Photoshoot",
                (dumpFactory.Shoot().alongWith(new RunHopper()).alongWith(new StartIntake()))
        // .until(() -> Feeder.getInstance().getfuelSensor())
        );

        NamedCommands.registerCommand("Climb",
                new PrintCommand("Hi!"));

    }

    public static double getMetersFromTarget(){
        Pose2d robotPose=CommandSwerveDrivetrain.getInstance().getState().Pose;
        Translation2d shooterPosition = robotPose.getTranslation();
        Pose2d target=isPassing ? Constants.FieldConstants.PassTarget
            :Constants.FieldConstants.HubPosition;

        
        return shooterPosition.getDistance(target.getTranslation());
    }

    public static void setShooterPass(boolean pass){
        shooterPass=pass;
    }

    public static void setShooterIsDumb(boolean isDumb){
        shooterIsDumb=isDumb;
    }
    public static boolean isPassing(){
        return shooterPass;
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