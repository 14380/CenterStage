package org.firstinspires.ftc.teamcode.commands.vertical;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.VerticalSlideSubsystem;

public class PosAExtendCommand extends CommandBase {

    private VerticalSlideSubsystem vertSlide;

    public PosAExtendCommand(VerticalSlideSubsystem vert){
        vertSlide = vert;
    }
    @Override
    public void initialize(){
        vertSlide.PositionAEx();
    }
    @Override
    public boolean isFinished(){

        return this.vertSlide.IsPositionAEx();
    }
}