package org.firstinspires.ftc.teamcode.commands.vertical;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.VerticalSlideSubsystem;

public class PosAutoExtendCommand extends CommandBase {

    private VerticalSlideSubsystem vertSlide;

    public PosAutoExtendCommand(VerticalSlideSubsystem vert){
        vertSlide = vert;
    }
    @Override
    public void initialize(){
        vertSlide.AutoPosition();
    }
    @Override
    public boolean isFinished(){

        return this.vertSlide.IsAutoPosition();
    }
}