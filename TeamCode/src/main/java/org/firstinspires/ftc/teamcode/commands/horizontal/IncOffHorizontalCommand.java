package org.firstinspires.ftc.teamcode.commands.horizontal;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlideSubsystem;

public class IncOffHorizontalCommand  extends CommandBase {

    private HorizontalSlideSubsystem slideSubsystem;

    public IncOffHorizontalCommand(HorizontalSlideSubsystem slide){
        this.slideSubsystem = slide;
    }

    @Override
    public void initialize(){

        slideSubsystem.ManualExtendOff();

    }

    @Override
    public boolean isFinished(){

        return true;
    }


}
