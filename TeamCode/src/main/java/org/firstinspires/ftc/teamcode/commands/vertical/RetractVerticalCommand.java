package org.firstinspires.ftc.teamcode.commands.vertical;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlideSubsystem;

public class RetractVerticalCommand extends CommandBase {

    private VerticalSlideSubsystem slideSubsystem;

    private RobotStateSubsystem rState;

    public RetractVerticalCommand(VerticalSlideSubsystem slide, RobotStateSubsystem state){
        this.slideSubsystem = slide;
        this.rState = state;
    }

    @Override
    public void initialize(){

        this.slideSubsystem.Retract();
    }
    @Override
    public boolean isFinished(){


        return this.slideSubsystem.IsRetracted();
    }
}
