package frc.robot.subsystems;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Iterator;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.spark.config.SmartMotionConfig;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class AddAutoSubsystem extends SubsystemBase{
    //TODO: Make the SmartDashboard add strings in the table.
    //Then, convert that to PathPlannerAuto making the string value the name of the auto.
    //Perhaps use this to allow users to select options: https://docs.wpilib.org/en/stable/docs/software/dashboards/smartdashboard/choosing-an-autonomous-program-from-smartdashboard.html
    //Use this for adding entries in https://www.geeksforgeeks.org/how-to-add-an-element-to-an-array-in-java/
    
    //ArrayList<PathPlannerAuto> pathList = new ArrayList<PathPlannerAuto>();

    ArrayList<PathPlannerAuto> pathList;

    //ArrayList<String> pathNames = new ArrayList<String>();
    //SendableChooser<Command> m_chooser = new SendableChooser<Command>();//New plan: use sendables
    public AddAutoSubsystem(ArrayList<PathPlannerAuto> path){
        pathList = path;
    }
    //String[] pathNames = {"Test auto"};
    /*public AddAutoSubsystem(){
        SmartDashboard.putStringArray("Autonomous Path",pathNames);
    }*/
    public void addPathToEnd(PathPlannerAuto path){//Adds path 
        pathList.add(path);//Adds path from the end
    }
    public void addPathUsingString(String path){//Adds path by taking in name of path.
        pathList.add(new PathPlannerAuto(path));
    }
    /*public void addPathNameToEnd(String path){
        String newArray[] = new String[pathNames.length+1];//I got this from a website. Don't ask me what this is.
        for(int i = 0; i < newArray.length; i++){
            newArray[i] = pathNames[i];
        }
        newArray[pathNames.length] = path;
    }*/
    public void addPathToCertainIndex(int index,PathPlannerAuto path){//Adds path in a certain index.
        pathList.add(index,path);
    }
    public void deletePathByPath(PathPlannerAuto path){
        pathList.remove(path);//Removes by path name. So if path 1 wants to be removed, the first instance of it will be removed. Not reliable if there are multiple instances of this. 
    }
    public void deletePathByIndex(int index){
        pathList.remove(index);//Removes by index of array. Guaranteed to remove the path desired if interface can do it like this.
    }
    public void replacePath(int index, PathPlannerAuto path){//Replaces path
        pathList.remove(index);
        pathList.add(index,path);
    }
    public void displayPaths(){
        String[] array = new String[pathList.size()]; 
        int num = 0;
        for(Iterator <PathPlannerAuto> i = pathList.iterator(); i.hasNext();){
            array[num] = i.next().getName();
            num+=1;
        }
        SmartDashboard.putStringArray("Path Names",array);//Can't see this in smardashboard
    }
    public void configurePathsAuto(String autoName){//Combines paths into a giant auto.
        SequentialCommandGroup newPathList = new SequentialCommandGroup();
        for(Iterator <PathPlannerAuto> i = pathList.iterator(); i.hasNext();){//Loops through the entire array
            newPathList.addCommands(i.next());
        }
        SmartDashboard.putData(autoName, newPathList);
        //return newPathList;
    }//This method will create a new SequentialCommandGroup based on the array by simply looping around it.
    public SequentialCommandGroup configurePaths(){//Combines paths into a giant auto.
        SequentialCommandGroup newPathList = new SequentialCommandGroup();
        for(Iterator <PathPlannerAuto> i = pathList.iterator(); i.hasNext();){//Loops through the entire array
            newPathList.addCommands(i.next());
        }
        return newPathList;
    }//This method will create a new SequentialCommandGroup based on the array by simply looping around it.

    /*public ArrayList<PathPlannerAuto> returnArray(){
        return pathList;
    }*/
    /*public void configureSendablePaths(){
        m_chooser.addDefaultOption("Test auto", pathList.get(0));//This will add the option to add the first index in pathlist.
        m_chooser.addOption("Test auto 2", pathList.get(1));
        //SmartDashboard.putData("Testing autos :O",m_chooser);
    }*/
}
