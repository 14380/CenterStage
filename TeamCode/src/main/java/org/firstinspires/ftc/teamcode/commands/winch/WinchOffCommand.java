package org.firstinspires.ftc.teamcode.commands.winch;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WinchSubsystem;

public class WinchOffCommand extends CommandBase {


    private WinchSubsystem winchSubsystem;

    public WinchOffCommand(WinchSubsystem winch){
        this.winchSubsystem = winch;
    }

    @Override
    public void initialize(){
        this.winchSubsystem.WinchOff();
    }



    @Override
    public boolean isFinished(){


        return true;
    }
}
