package org.firstinspires.ftc.teamcode.commands.winch;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WinchSubsystem;

public class DeployRightHookCommand extends CommandBase {


    private WinchSubsystem winchSubsystem;

    public DeployRightHookCommand(WinchSubsystem winch){
        this.winchSubsystem = winch;
    }

    @Override
    public void initialize(){
        this.winchSubsystem.DeployRightHooks();
    }



    @Override
    public boolean isFinished(){


        return true;
    }
}
