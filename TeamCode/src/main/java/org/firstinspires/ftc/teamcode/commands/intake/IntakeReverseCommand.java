package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeReverseCommand extends CommandBase {

    private IntakeSubsystem intakeSubsystem;

    public IntakeReverseCommand(IntakeSubsystem intake){
        this.intakeSubsystem = intake;
    }

    @Override
    public void initialize(){
        this.intakeSubsystem.Reverse();
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
