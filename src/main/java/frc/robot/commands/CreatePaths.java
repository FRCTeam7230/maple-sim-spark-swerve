package frc.robot.commands;

import frc.robot.subsystems.AddAutoSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class CreatePaths {
    private AddAutoSubsystem m_auto;
    public CreatePaths(AddAutoSubsystem auto){
        m_auto = auto;
    }
    public void makeThePaths(){//Add this as an array for later?
        m_auto.addPathUsingString("Test auto");
    }
}
