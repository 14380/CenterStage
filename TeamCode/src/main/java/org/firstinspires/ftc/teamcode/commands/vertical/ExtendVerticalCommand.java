package org.firstinspires.ftc.teamcode.commands.vertical;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlideSubsystem;

public class ExtendVerticalCommand extends CommandBase {

    private VerticalSlideSubsystem slideSubsystem;

    public ExtendVerticalCommand(VerticalSlideSubsystem slide){
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
