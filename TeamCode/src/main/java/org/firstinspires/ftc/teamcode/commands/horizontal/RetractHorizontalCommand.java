package org.firstinspires.ftc.teamcode.commands.horizontal;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class RetractHorizontalCommand extends CommandBase {

    private HorizontalSlideSubsystem slideSubsystem;

    public RetractHorizontalCommand(HorizontalSlideSubsystem slide){
        this.slideSubsystem = slide;
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
