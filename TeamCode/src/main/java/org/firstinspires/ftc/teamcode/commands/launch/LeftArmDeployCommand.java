package org.firstinspires.ftc.teamcode.commands.launch;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WinchSubsystem;

public class LeftArmDeployCommand extends CommandBase {

    private WinchSubsystem launcherSubsystem;

    public LeftArmDeployCommand(WinchSubsystem launch){
        launcherSubsystem = launch;
    }

    @Override
    public void initialize(){

        this.launcherSubsystem.DeployLeftHooks();
    }
    @Override
    public boolean isFinished(){


        return true;
    }

}