package org.firstinspires.ftc.teamcode.commands.horizontal;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlideSubsystem;

public class IncExtendHorizontalCommand extends CommandBase {

    private HorizontalSlideSubsystem slideSubsystem;

    public IncExtendHorizontalCommand(HorizontalSlideSubsystem slide){
        this.slideSubsystem = slide;
    }

    @Override
    public void initialize(){

        slideSubsystem.ManualExtend(1);

    }

    @Override
    public boolean isFinished(){

        return this.slideSubsystem.IsExtended();
    }


}
