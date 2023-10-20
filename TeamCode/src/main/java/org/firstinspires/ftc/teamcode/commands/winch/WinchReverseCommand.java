package org.firstinspires.ftc.teamcode.commands.winch;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WinchSubsystem;

public class WinchReverseCommand extends CommandBase {


    private WinchSubsystem winchSubsystem;

    public WinchReverseCommand(WinchSubsystem winch){
        this.winchSubsystem = winch;
    }

    @Override
    public void initialize(){
        this.winchSubsystem.WinchReverse();
    }



    @Override
    public boolean isFinished(){


        return true;
    }
}
