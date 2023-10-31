package org.firstinspires.ftc.teamcode.commands.winch;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WinchSubsystem;

public class DeployLeftHookCommand extends CommandBase {


    private WinchSubsystem winchSubsystem;

    public DeployLeftHookCommand(WinchSubsystem winch){
        this.winchSubsystem = winch;
    }

    @Override
    public void initialize(){
        this.winchSubsystem.DeployLeftHooks();
    }



    @Override
    public boolean isFinished(){


        return true;
    }
}
