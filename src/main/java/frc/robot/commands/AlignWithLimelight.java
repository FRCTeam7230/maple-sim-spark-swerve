package frc.robot.commands;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
public class AlignWithLimelight extends Command {
    Drive m_drive;
    Vision m_limelight;
    ElevatorSubsystem m_elevator;

    LimelightConstants.reefAlignSide alignSide;

    double horizontalOffset;
    double forwardOffset;

    PIDController xController;
    PIDController forwardController;
    PIDController yawController;
    LinearFilter filter= LinearFilter.movingAverage(5);
    public AlignWithLimelight(Drive drive, Vision limelight, LimelightConstants.reefAlignSide side) {
        m_drive = drive;
        m_limelight = limelight;
        alignSide = side;

        xController = new PIDController(LimelightConstants.kDriveHorizontalKp,0.3,0);
        forwardController = new PIDController(LimelightConstants.kDriveForwardKp,0.3,0);
        yawController = new PIDController(LimelightConstants.kRotationKp,0,0.002);
    }
    List<Pose3d> arr;
    int index;
    @Override
    public void initialize() {
        if(alignSide == LimelightConstants.reefAlignSide.Left)
        {
        horizontalOffset = -1*LimelightConstants.kHorizontalOffset;
        }
        else
        {
        horizontalOffset = LimelightConstants.kHorizontalOffset;
        }
        forwardOffset = LimelightConstants.kForwardUnextendedOffset;

        arr = m_limelight.getAllTags(0);
        Translation2d robotLocation = m_drive.getPose().getTranslation();
        if (arr.size()>0){
            for (Pose3d i : arr){
                Translation2d dist = new Translation2d(i.getX(),i.getY());
                if (dist.getDistance(robotLocation)<new Translation2d(arr.get(index).getX(),arr.get(index).getY()).getDistance(robotLocation)){
                    index = arr.indexOf(i);
                }
            }
            double oldH = horizontalOffset;
            double oldF = forwardOffset;
            double angle = arr.get(index).getRotation().toRotation2d().getRadians()+Math.PI/2;
            horizontalOffset = Math.cos(angle)*oldH-Math.sin(angle)*oldF;//carried by chatgpt bcz i didn't know rotational matrix
            forwardOffset = Math.sin(angle)*oldH+Math.cos(angle)*oldF;
            Logger.recordOutput("Vision/Aligning/Chosen Tag", arr.get(index));
            Logger.recordOutput("Vision/Aligning/Align Loscation", new Pose2d(new Translation2d(horizontalOffset+arr.get(index).getX(), forwardOffset+arr.get(index).getY()),arr.get(index).getRotation().toRotation2d().plus(new Rotation2d(Math.PI))));
        }

        xController.setTolerance(LimelightConstants.kPositionErrorThreshold);
        forwardController.setTolerance(LimelightConstants.kPositionErrorThreshold);
        yawController.setTolerance(LimelightConstants.kRotationErrorThreshold);

        xController.setSetpoint(horizontalOffset);
        forwardController.setSetpoint(forwardOffset);
        yawController.setSetpoint(180);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {//The robot switches tags, so move the tag chooser to the intialize method. 
        if (arr.size()>0){
            
            //Pose2d robotRelativeToTag = m_drive.getPose().relativeTo(arr.get(index).toPose2d());
             Pose2d robotRelativeToTag = new Pose2d(
                 new Translation2d(m_drive.getPose().getX()-arr.get(index).getX(),
                 m_drive.getPose().getY()-arr.get(index).getY()),
                 new Rotation2d(m_drive.getPose().getRotation().getRadians()-arr.get(index).getRotation().toRotation2d().getRadians()));
            double tx = robotRelativeToTag.getX();
            double ty = robotRelativeToTag.getY();
            double yaw = robotRelativeToTag.getRotation().getDegrees();
            double xValue = -xController.calculate(tx);
            double yValue = -forwardController.calculate(ty);
            double yawValue = yawController.calculate(yaw);

            Logger.recordOutput("Vision/Aligning/robot location", new Pose2d(new Translation2d(tx+arr.get(index).getX(),ty+arr.get(index).getY()), new Rotation2d((yaw+arr.get(index).getRotation().toRotation2d().getDegrees())*Math.PI/180)));
            Logger.recordOutput("Vision/Aligning/x", xValue);
            Logger.recordOutput("Vision/Aligning/y", yValue);
            Logger.recordOutput("Vision/Aligning/yaw", yawValue);
            
            DriveCommands.joystickDrive(m_drive,()->xValue,()->yValue,()->yawValue).schedule();
            //DriveCommands.robotJoystickDrive(m_drive,xValue,yValue,yawValue).schedule();
            //I think this works because the x and y of this command should be the same as the x and y I inputted for robot and tags. It's the same field that we're using.
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        DriveCommands.robotJoystickDrive(m_drive,0, 0, 0).schedule();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if(xController.atSetpoint()&&yawController.atSetpoint()&&forwardController.atSetpoint())
        {
            return true;
        }
        return false;
    }
}
