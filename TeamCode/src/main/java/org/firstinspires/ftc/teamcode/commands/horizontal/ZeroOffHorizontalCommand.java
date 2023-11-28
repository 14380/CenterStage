package org.firstinspires.ftc.teamcode.commands.horizontal;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlideSubsystem;

public class ZeroOffHorizontalCommand extends CommandBase {

    private HorizontalSlideSubsystem slideSubsystem;

    public ZeroOffHorizontalCommand(HorizontalSlideSubsystem slide){
        this.slideSubsystem = slide;
    }

    @Override
    public void initialize(){

        slideSubsystem.ManualZeroPower();

    }

    @Override
    public boolean isFinished(){

        return true;
    }


}
