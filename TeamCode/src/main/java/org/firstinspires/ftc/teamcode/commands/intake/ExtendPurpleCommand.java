package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class ExtendPurpleCommand extends CommandBase {

    private IntakeSubsystem intakeSubsystem;

    public ExtendPurpleCommand(IntakeSubsystem intake){
        this.intakeSubsystem = intake;
    }

    @Override
    public void initialize(){

        this.intakeSubsystem.ExtendPurple();
    }
    @Override
    public boolean isFinished(){

        return true;
    }

}
