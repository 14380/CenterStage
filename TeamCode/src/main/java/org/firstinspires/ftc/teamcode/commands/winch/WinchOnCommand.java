package org.firstinspires.ftc.teamcode.commands.winch;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WinchSubsystem;

public class WinchOnCommand extends CommandBase {


    private WinchSubsystem winchSubsystem;

    public WinchOnCommand(WinchSubsystem winch){
        this.winchSubsystem = winch;
    }

    @Override
    public void initialize(){
        this.winchSubsystem.WinchOn();
    }



    @Override
    public boolean isFinished(){


        return true;
    }
}
