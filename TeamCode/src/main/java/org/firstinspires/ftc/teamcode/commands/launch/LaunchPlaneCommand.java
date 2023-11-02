package org.firstinspires.ftc.teamcode.commands.launch;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

public class LaunchPlaneCommand extends CommandBase {

    private LauncherSubsystem launcherSubsystem;

    public LaunchPlaneCommand(LauncherSubsystem launch){
        launcherSubsystem = launch;
    }

    @Override
    public void initialize(){

        this.launcherSubsystem.LaunchPlane();
    }

    @Override
    public boolean isFinished(){


        return true;
    }

}