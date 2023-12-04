package org.firstinspires.ftc.teamcode.commands.vertical;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.VerticalSlideSubsystem;

public class PosAutoExExtendCommand extends CommandBase {

    private VerticalSlideSubsystem vertSlide;

    public PosAutoExExtendCommand(VerticalSlideSubsystem vert){
        vertSlide = vert;
    }
    @Override
    public void initialize(){
        vertSlide.PositionAutoEx();
    }
    @Override
    public boolean isFinished(){

        return this.vertSlide.IsPositionAutoEx();
    }
}