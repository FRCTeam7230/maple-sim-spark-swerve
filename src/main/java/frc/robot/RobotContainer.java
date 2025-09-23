// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;
//import frc.robot.commands.AlignWithLimelightSim;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommand;
import frc.robot.subsystems.AddEmergencyPathFinding;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.AIRobotInSimulation;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final Vision vision;
    private SwerveDriveSimulation driveSimulation = null;

    private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();


    // Controller
    private final Joystick controller = new Joystick(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    //private LoggedDashboardChooser<Command> m_chooser;
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        

        

        
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOSpark(0),
                        new ModuleIOSpark(1),
                        new ModuleIOSpark(2),
                        new ModuleIOSpark(3),
                        (pose) -> {});

                this.vision = new Vision(
                        drive,
                        new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                        new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));

                break;
            case SIM:
                m_elevator.setGoal(0);
                // create a maple-sim swerve drive simulation instance
                this.driveSimulation =
                        new SwerveDriveSimulation(DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
                // add the simulated drivetrain to the simulation field
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOSim(driveSimulation.getModules()[0]),
                        new ModuleIOSim(driveSimulation.getModules()[1]),
                        new ModuleIOSim(driveSimulation.getModules()[2]),
                        new ModuleIOSim(driveSimulation.getModules()[3]),
                        driveSimulation::setSimulationWorldPose);

                drive.driveSimulation = driveSimulation;

                vision = new Vision(
                        drive,
                        new VisionIOPhotonVisionSim(
                                camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                        new VisionIOPhotonVisionSim(
                                camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));
                
                
                AIRobotInSimulation.startOpponentRobotSimulations();

                break;
            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        (pose) -> {});
                vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});

                break;
        }

        //TODO: Test this on advantage scope.

        ElevatorCommand elevUp = new ElevatorCommand(m_elevator,Constants.ElevatorConstants.kMaxElevatorHeightMeters);
        ElevatorCommand elevDown = new ElevatorCommand(m_elevator,Constants.ElevatorConstants.kMinElevatorHeightMeters);
        //ProxyCommand a = new ProxyCommand(elevUp);//What if we use proxy?
        ElevatorCommand score = new ElevatorCommand(m_elevator,Constants.ElevatorConstants.kScoreElevatorHeightMeters);
        
        ElevatorCommand elevUpToL3 = new ElevatorCommand(m_elevator,Constants.ElevatorConstants.kMaxElevatorHeightMeters-1);
        ElevatorCommand L3Score = new ElevatorCommand(m_elevator, Constants.ElevatorConstants.kMaxElevatorHeightMeters-1.1);
        //CreatePaths path = new CreatePaths(auto);
        //path.makeThePaths();
        //Pathfinding basics. Finds a path from current position to a specific coordinate. Use this as last resort if
        //Vision alignment doesn't work for some reason or the robot is a bit shifted at the intake for some reason.
        
        
        //DriverStation.Alliance ally = DriverStation.getAlliance().get(); 
        /*Translation2d a = new Translation2d(
                DriverStation.getAlliance().get()==DriverStation.Alliance.Blue?coordinateX:17.54-coordinateX,
                DriverStation.getAlliance().get()==DriverStation.Alliance.Blue?coordinateY:8-coordinateY);*/
        /*Pose2d target = new Pose2d(new Translation2d(
                DriverStation.getAlliance().get()==DriverStation.Alliance.Blue?coordinateX:17.54-coordinateX,
                DriverStation.getAlliance().get()==DriverStation.Alliance.Blue?coordinateY:8-coordinateY),
                new Rotation2d(180));
        PathConstraints constraints = new PathConstraints(3, 3, 540, 720);
        Command pathFindingCommand = AutoBuilder.pathfindToPose(
                target,
                constraints,
                0.0 // Goal end velocity in meters/sec
                //0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );*///DriverStation crashes the code for some reason. The rest works just fine as seen below.
        
        /*String ally = "Red";
        Pose2d target = new Pose2d(new Translation2d(
                ally=="Blue"?coordinateX:17.54-coordinateX,
                ally=="Blue"?coordinateY:8-coordinateY),
                new Rotation2d(Math.PI));
        PathConstraints constraints = new PathConstraints(3, 3, 540, 720);
        Command pathFindingCommand = AutoBuilder.pathfindToPose(
                target,
                constraints,
                0.0 // Goal end velocity in meters/sec
                //0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );*/
        //AlignWithLimelightSim visionAlignAndScoreLeft  = new AlignWithLimelightSim(drive.getPose()); //TODO Replace with set of commands to align, score and drive backwards
        WaitCommand visionAlignAndScoreRight = new WaitCommand(1.5); //TODO Replace with set of commands to align, score and drive backwards
        /*SequentialCommandGroup visionAlignAndScoreLeft = new SequentialCommandGroup();
        visionAlignAndScoreLeft.addCommands(VisionAlignCommandLeft);
        visionAlignAndScoreLeft.addCommands(score);
        visionAlignAndScoreLeft.addCommands(m_robotDrive.drive(0,-1,0,false));

        SequentialCommandGroup visionAlignAndScoreRight = new SequentialCommandGroup();
        visionAlignAndScoreRight.addCommands(VisionAlignCommandRight);
        visionAlignAndScoreRight.addCommands(score);
        visionAlignAndScoreRight.addCommands(m_robotDrive.drive(0,-1,0,false));
        */
        double coordinateX = 1.2;
        double coordinateY = 1.1;
        AddEmergencyPathFinding testPath2 = new AddEmergencyPathFinding(coordinateX, coordinateY, 180,"Red");//angle is in degrees.
        testPath2.registerCommand("Last Align");//Register the pathfinding to a command.
        
        NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
    NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
    NamedCommands.registerCommand("print hello", Commands.print("hello"));
    NamedCommands.registerCommand("Lift the Elevator",new WaitCommand(5));//We can add commands like this, and yes it works as long as you can bear the 5 second wait.
    NamedCommands.registerCommand("Dance", Commands.print("This will not be a command where the robot will spin around itself."));

    NamedCommands.registerCommand("Raise Elevator",elevUp);
    NamedCommands.registerCommand("Lower Elevator",elevDown);
    NamedCommands.registerCommand("Score",score);
    NamedCommands.registerCommand("Shoot Alage", Commands.runOnce(drive::scoreAlgae, drive));
    //NamedCommands.registerCommand("Vision Align and Score Left", Commands.run(visionAlignAndScoreLeft.followPathToReef()))
    NamedCommands.registerCommand("Vision Align and Score Left", new WaitCommand(0.1));
   /// SmartDashboard.
    //SmartDashboard.putNumber("Testing AprilTag detection sim",visionAlignAndScoreLeft.findNearestAprilTag());


    //NamedCommands.registerCommand("Last Align",testPath2.runPathCommand());
        
        //So this command does run an auto as we desired. However, when making an auto with these commands, YOU MUST ADD A PATH BEFORE THAT. OTHERWISE IT WILL CRASH.
        //SequentialCommandGroup test = new SequentialCommandGroup();
        //test.addCommands(new PathPlannerAuto("COMP - Start Center to Right (Our Barge) Coral Station"));
        //test.addCommands(t);
        NamedCommands.registerCommand("Testing that 1 command that should run an auto", new PathPlannerAuto("COMP - Start Center to Right (Our Barge) Coral Station"));

        // Use event markers as triggers
        //new EventTrigger("Example Marker").onTrue(Commands.print("Passed an event marker"));
        //new EventTrigger("Dance").onTrue(Commands.print("This will not be a command where the robot will spin around itself."));
        //new EventTrigger("Raise Elevator").onTrue(elevUp);
        //new EventTrigger("Lower Elevator").onTrue(elevDown);
        //autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        //SmartDashboard.putData("Auto Mode", autoChooser);

        
        
        PathPlannerAuto[][] listOfAuto = {//Paths can be stored here.
                {
                        new PathPlannerAuto("Testing that 1 command that should run the auto")
                },
                {
                        new PathPlannerAuto("Start Right Side Part 1"),
                        new PathPlannerAuto("Start Right Side Part 2"),
                },
                {
                        new PathPlannerAuto("Start Left Side Part 1"),
                        new PathPlannerAuto("Start Left Side Part 2"),
                },
                {
                        new PathPlannerAuto("Pathfinding"),
                },
        };

        
        
  
        //auto.configurePathsAuto("This may or may not work");
        /*auto.addPathToEnd(
                new PathPlannerAuto("COMP - Start Center to Right (Our Barge) Coral Station")
        );*/
        //SequentialCommandGroup fullAuto = new SequentialCommandGroup();
        //SequentialCommandGroup fullAuto = auto.configurePaths();
        //SequentialCommandGroup fullAuto = autos[0].configurePaths();
        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        SmartDashboard.putData("Reef 1 to Station Left", new PathPlannerAuto("Reef 1 to Coral Station Left"));
        SmartDashboard.putData("Reef 1 to Station Right", new PathPlannerAuto("Reef 1 to Station Right"));
        SmartDashboard.putData("Reef 2 to Station Right", new PathPlannerAuto("Reef 2 to Station Right"));
        SmartDashboard.putData("Reef 2 to Station Left", new PathPlannerAuto("Reef 2 to Station Left"));
        SmartDashboard.putData("Reef 3 to Station Top", new PathPlannerAuto("Reef 3 to Station Top"));
        SmartDashboard.putData("Reef 3 to Station Bottom", new PathPlannerAuto("Reef 3 to Station Bottom"));
        SmartDashboard.putData("Reef 5 to station right", new PathPlannerAuto("Reef 5 to station right"));
        SmartDashboard.putData("Reef 5 to station left", new PathPlannerAuto("Reef 5 to station left"));
        SmartDashboard.putData("Reef 6 to station right", new PathPlannerAuto("Reef 6 to station right"));
        SmartDashboard.putData("Reef 6 to station left", new PathPlannerAuto("Reef 6 to station left"));
        
        SmartDashboard.putData("Coral 6 Path 3", new PathPlannerAuto("Coral 6 Path 3"));
        SmartDashboard.putData("Coral 6 Path 2", new PathPlannerAuto("Coral 6 Path 2"));
        SmartDashboard.putData("Coral 5 Cycle RIght", new PathPlannerAuto("Coral 5 Cycle RIght"));
        SmartDashboard.putData("Reef 5 to Station left", new PathPlannerAuto("Reef 5 to Station left"));
        SmartDashboard.putData("Coral 5 Cycle 1", new PathPlannerAuto("Coral 5 Cycle 1"));

        SmartDashboard.putData("Coral Cycle 1", new PathPlannerAuto("Coral Cycle 1"));
        SmartDashboard.putData("Coral 1 Cycle 2", new PathPlannerAuto("Coral 1 Cycle 2"));
        SmartDashboard.putData("Coral Cycle 2", new PathPlannerAuto("Coral Cycle 2"));
        //SmartDashboard.putData("Coral Cycle 1", new PathPlannerAuto("Coral Cycle 1"));
        SmartDashboard.putData("Coral 3 Cycle", new PathPlannerAuto("Coral 3 Cycle"));
        //SmartDashboard.putData("Coral Cycle 1", new PathPlannerAuto("Coral Cycle 1"));
        SmartDashboard.putData("Coral 4 Cycle", new PathPlannerAuto("Coral Cycle"));
        SmartDashboard.putData("Coral 4 Cycle 2", new PathPlannerAuto("Coral Cycle 2"));
        SmartDashboard.putData("Coral 5 Cycle", new PathPlannerAuto("Coral 5 Cycle"));
        SmartDashboard.putData("Coral 5 Cycle 2", new PathPlannerAuto("Coral 5 Cycle 2"));
        SmartDashboard.putData("Coral 6 Cycle", new PathPlannerAuto("Coral 6 Cycle 1"));
        //SmartDashboard.putData("Coral 6 Cycle 1", new PathPlannerAuto("Coral 6 Cycle 1"));
        SmartDashboard.putData("Test auto", new PathPlannerAuto("Test auto"));
        SmartDashboard.putData("COMP - Start Left (Processor) Side", new PathPlannerAuto("COMP - Start Left (Processor) Side"));
        
        SmartDashboard.putData("COMP - Start Center to Right (Our Barge) Coral Station", new PathPlannerAuto("COMP - Start Center to Right (Our Barge) Coral Station"));
        SmartDashboard.putData("COMP - Start Right (Our Barge) Side Auto", new PathPlannerAuto("COMP - Start Right (Our Barge) Side Auto"));
        SmartDashboard.putData("Spawn Coral", Commands.runOnce(drive::scoreCoral, drive));
        SmartDashboard.putData("Drop Algae", Commands.runOnce(drive::scoreAlgae, drive));
        SmartDashboard.putData("Start Intake", Commands.runOnce(drive::intakeCoralStart, drive));
        SmartDashboard.putData("Stop Intake", Commands.runOnce(drive::intakeCoralStop, drive));
        SmartDashboard.putData("Drop Coral",
                //new RunCommand(() -> controller.setOutput(1, true))
                //.andThen(
                        new RunCommand(() -> SmartDashboard.putBoolean("is the button pressed?",controller.getRawButton(1)))
                        //)
        );
        SmartDashboard.putData("Clear entities", Commands.runOnce(drive::removeAllEntities,drive));
        
        Runnable moveToSpawner1 = () -> drive.giveCoralFromSpawner(new Translation2d(0.8,7.7),90-55+90,-50);
        Runnable moveToSpawner2 = () -> drive.giveCoralFromSpawner(new Translation2d(17.55-0.8,8.05-7.7),-90-55,-50);
        Runnable moveToSpawner3 = () -> drive.giveCoralFromSpawner(new Translation2d(0.8,8.05-7.7),180-45,220);
        Runnable moveToSpawner4 = () -> drive.giveCoralFromSpawner(new Translation2d(17.55-0.8,7.7),180-(-90-55),220);
        
        //create a command 
        
        //Add a new command in smartdashboard

        SequentialCommandGroup spawnDaCorals = new SequentialCommandGroup(
                Commands.runOnce(moveToSpawner1,drive),
                Commands.runOnce(moveToSpawner2,drive),
                Commands.runOnce(moveToSpawner3,drive),
                Commands.runOnce(moveToSpawner4,drive)
        );
        SmartDashboard.putData("Run intake",
                spawnDaCorals
        );
        SmartDashboard.putData("Stop intake",Commands.runOnce(() -> new WaitCommand(0)));

        


        //SmartDashboard.putData("COMP - ",fullAuto);
        
        //NamedCommands.registerCommand("An auto",new PathPlannerAuto("COMP - Start Center to Right (Our Barge) Coral Station"));
        /*int autoCount = listOfAuto.length;
        SequentialCommandGroup[] autos = new SequentialCommandGroup[autoCount];
        for (int i = 0; i < listOfAuto.length; i++){
                autos[i] = new SequentialCommandGroup();
                for (int j = 0; j < listOfAuto[i].length; j++){
                autos[i].addCommands(listOfAuto[i][j]);
                }
                SmartDashboard.putData("Auto "+Integer.toString(i),autos[i]);
                //SmartDashboard.setPersistent("Auto "+Integer.toString(i));
        }*/
        
        /*autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        (stream) -> isCompetition
            ? stream.filter(auto -> auto.getName().startsWith("COMP"))
            : stream
        );*/
        //autoChooser.addDefaultOption("Auto 0", autos[0]);//This will set the default auto that will run.
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */

    double speedMult = 0.75;
    double rotMult = 0.65;
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        // getX moves left/right.
        // getY moves up/down.
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive, () -> controller.getRawAxis(1) * speedMult, () -> controller.getRawAxis(0) * speedMult, () -> -controller.getRawAxis(2) * rotMult));


        double slowSpeed = 0.4;

        new JoystickButton(controller, 3)
                .whileTrue(DriveCommands.robotJoystickDrive(drive, 0, slowSpeed, 0));

                
        new JoystickButton(controller, 4)
        .whileTrue(DriveCommands.robotJoystickDrive(drive, 0, -slowSpeed, 0));
        new JoystickButton(controller, 5)
        .whileTrue(DriveCommands.robotJoystickDrive(drive, slowSpeed, 0, 0));
        new JoystickButton(controller, 6)
        .whileTrue(DriveCommands.robotJoystickDrive(drive, -slowSpeed, 0, 0));

        // Lock to 0° when A button is held
        // new JoystickButton(controller, 3)
        //         .whileTrue(DriveCommands.joystickDriveAtAngle(
        //                 drive, () -> controller.getY(), () -> controller.getX(), () -> new Rotation2d()));

        // Switch to X pattern when X button is pressed
        // new JoystickButton(controller, 4).onTrue(Commands.runOnce(drive::stopWithX, drive));
        // new JoystickButton(controller, 1).onTrue(Commands.runOnce(drive::scoreAlgae, drive));
        new JoystickButton(controller, 1).onTrue(Commands.runOnce(drive::scoreCoral, drive));
        // new JoystickButton(controller, 5).onTrue(Commands.runOnce(drive::spawnAlgae, drive));
        // new JoystickButton(controller, 6).onTrue(Commands.runOnce(drive::spawnCoral, drive));

        new JoystickButton(controller, 7)
                .whileTrue(new RunCommand(
                () -> m_elevator.reachGoal(Constants.ElevatorConstants.kMinElevatorHeightMeters),
                m_elevator));

        new JoystickButton(controller, 8)
                .whileTrue(new RunCommand(
                () -> m_elevator.reachGoal(Constants.ElevatorConstants.kMaxElevatorHeightMeters),
                m_elevator));

        // Reset gyro / odometry
        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
                ? () -> drive.resetOdometry(
                        driveSimulation
                                .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
                : () -> drive.resetOdometry(
                        new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro

        new JoystickButton(controller, 2)
                .onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    
    public Command getAutonomousCommand() {
        
        //SmartDashboard.putData("Testing autos :O",m_chooser);
        //return m_chooser.get();
        return autoChooser.get();
    }

    public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        drive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void displaySimFieldToAdvantageScope() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));

        Logger.recordOutput("FieldSimulation/OpponentRobotPositions", AIRobotInSimulation.getOpponentRobotPoses());
        Logger.recordOutput(
                "FieldSimulation/AlliancePartnerRobotPositions", AIRobotInSimulation.getAlliancePartnerRobotPoses());
    }
}
