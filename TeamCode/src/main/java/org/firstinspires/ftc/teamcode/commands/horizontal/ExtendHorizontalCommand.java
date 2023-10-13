package org.firstinspires.ftc.teamcode.commands.horizontal;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlideSubsystem;

public class ExtendHorizontalCommand extends CommandBase {

    private HorizontalSlideSubsystem slideSubsystem;

    public ExtendHorizontalCommand(HorizontalSlideSubsystem slide){
        this.slideSubsystem = slide;
    }

    @Override
    public void initialize(){

        this.slideSubsystem.Extend();
    }
    @Override
    public boolean isFinished(){

        return this.slideSubsystem.IsExtended();
    }
}
