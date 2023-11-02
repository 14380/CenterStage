package org.firstinspires.ftc.teamcode.commands.launch;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WinchSubsystem;

public class RightArmDeployCommand extends CommandBase {

    private WinchSubsystem launcherSubsystem;

    public RightArmDeployCommand(WinchSubsystem launch){
        launcherSubsystem = launch;
    }

    @Override
    public void initialize(){

        this.launcherSubsystem.DeployRightHooks();
    }

    @Override
    public boolean isFinished(){


        return true;
    }

}
