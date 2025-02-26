package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

public class AddEmergencyPathFinding extends SubsystemBase{
    double coordinateX;
    double coordinateY;
    Pose2d target;
    PathConstraints constraints;
    Command pathFindingCommand;
    public AddEmergencyPathFinding(double destinationX, double destinationY, double destinationAngle, String alliance){
        coordinateX = destinationX;//3.9
        coordinateY = destinationY;//3.3/
        /*target = new Pose2d(new Translation2d(
                DriverStation.getAlliance().get()==DriverStation.Alliance.Blue?coordinateX:17.54-coordinateX,
                DriverStation.getAlliance().get()==DriverStation.Alliance.Blue?coordinateY:8-coordinateY),
                new Rotation2d(destinationAngle));*/
        //For whatever reason, using DriverStation class crashes my code, so use this instead.

        target = new Pose2d(new Translation2d(
            alliance=="Blue"?coordinateX:17.54-coordinateX,
            alliance=="Blue"?coordinateY:8-coordinateY),
            new Rotation2d(Math.toRadians(destinationAngle)));
        
        constraints = new PathConstraints(3, 3, 540, 720);//Global constraints for robot, change this if different
        pathFindingCommand = AutoBuilder.pathfindToPose(
                target,
                constraints,
                0.0 // Goal end velocity in meters/sec
                //0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );

    }
    public Command runPathCommand(){
        return pathFindingCommand;
    }
    public void registerCommand(String name){
            NamedCommands.registerCommand(name,runPathCommand());//Register the pathfinding to a command.
    }
}
