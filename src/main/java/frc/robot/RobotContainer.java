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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.vision.*;
import frc.robot.util.AIRobotInSimulation;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import frc.robot.Constants.*;
import frc.robot.subsystems.elevator.*;
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

        SequentialCommandGroup score = new SequentialCommandGroup();
        score.addCommands(new RunCommand(
                () -> m_elevator.reachGoal(Constants.ElevatorConstants.kMaxElevatorHeightMeters),
                m_elevator));
        score.addCommands(Commands.runOnce(drive::scoreCoral, drive));
        score.addCommands(new RunCommand(
                () -> m_elevator.reachGoal(Constants.ElevatorConstants.kMinElevatorHeightMeters),
                m_elevator));

        NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
        NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
        NamedCommands.registerCommand("print hello", Commands.print("hello"));
        /*NamedCommands.registerCommand("Lift the Elevator",(new RunCommand(
                () -> m_elevator.reachGoal(Constants.ElevatorSimConstants.kMaxElevatorHeightMeters),
                m_elevator)));//We can add commands like this, and yes it works as long as you can bear the 5 second wait.*/
        NamedCommands.registerCommand("Lift the Elevator",(score));//We can add commands like this, and yes it works as long as you can bear the 5 second wait.
        //NamedCommands.registerCommand("Shoot Coral",(Commands.runOnce(drive::scoreCoral, drive)));
        //NamedCommands.registerCommand("Lift the Elevator",(Commands.runOnce(drive::scoreCoral, drive)));
        //NamedCommands.registerCommand("Lower the Elevator",(Commands.runOnce(new WaitCommand(1))));
        NamedCommands.registerCommand("Dance", Commands.print("This will not be a command where the robot will spin around itself."));


        // Use event markers as triggers
        new EventTrigger("Example Marker").onTrue(Commands.print("Passed an event marker"));
        new EventTrigger("Dance").onTrue(Commands.print("This will not be a command where the robot will spin around itself."));

        //autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        //SmartDashboard.putData("Auto Mode", autoChooser);


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

        SmartDashboard.putData("Reef 1 to Coral Station Left", new PathPlannerAuto("Reef 1 to Coral Station Left"));
        SmartDashboard.putData("Reef 1 to Station Right", new PathPlannerAuto("Reef 1 to Station Right"));
        SmartDashboard.putData("Reef 2 to Station Right", new PathPlannerAuto("Reef 2 to Station Right"));
        SmartDashboard.putData("Reef 2 to Station Left", new PathPlannerAuto("Reef 2 to Station Left"));
        SmartDashboard.putData("Reef 3 to Station Top", new PathPlannerAuto("Reef 3 to Station Top"));
        SmartDashboard.putData("Reef 3 to Station Bottom", new PathPlannerAuto("Reef 3 to Station Bottom"));
        SmartDashboard.putData("Reef 5 to station right", new PathPlannerAuto("Reef 5 to station right"));
        SmartDashboard.putData("Reef 5 to station left", new PathPlannerAuto("Reef 5 to station left"));
        SmartDashboard.putData("Reef 6 to station right", new PathPlannerAuto("Reef 6 to station right"));
        SmartDashboard.putData("Reef 6 to station left", new PathPlannerAuto("Reef 6 to station left"));

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

        new JoystickButton(controller, 11).whileTrue(DriveCommands.toggleDrive());

        // double slowSpeed = 0.4;
        // new JoystickButton(controller, 3)
        //         .whileTrue(DriveCommands.robotJoystickDrive(drive, 0, slowSpeed, 0));

                
        // new JoystickButton(controller, 4)
        // .whileTrue(DriveCommands.robotJoystickDrive(drive, 0, -slowSpeed, 0));
        // new JoystickButton(controller, 5)
        // .whileTrue(DriveCommands.robotJoystickDrive(drive, slowSpeed, 0, 0));
        // new JoystickButton(controller, 6)
        // .whileTrue(DriveCommands.robotJoystickDrive(drive, -slowSpeed, 0, 0));

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
