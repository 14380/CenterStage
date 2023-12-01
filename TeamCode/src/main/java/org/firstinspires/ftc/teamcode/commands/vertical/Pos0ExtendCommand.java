package org.firstinspires.ftc.teamcode.commands.vertical;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.VerticalSlideSubsystem;

public class Pos0ExtendCommand extends CommandBase {

    private VerticalSlideSubsystem vertSlide;

    public Pos0ExtendCommand(VerticalSlideSubsystem vert){
        vertSlide = vert;
    }
    @Override
    public void initialize(){
        vertSlide.Position0();
    }
    @Override
    public boolean isFinished(){

        return this.vertSlide.IsPosition0();
    }
}