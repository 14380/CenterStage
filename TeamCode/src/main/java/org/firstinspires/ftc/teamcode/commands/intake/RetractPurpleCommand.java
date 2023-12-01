package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class RetractPurpleCommand extends CommandBase {

    private IntakeSubsystem intakeSubsystem;

    public RetractPurpleCommand(IntakeSubsystem intake){
        this.intakeSubsystem = intake;
    }

    @Override
    public void initialize(){

        this.intakeSubsystem.RetractPurple();
    }
    @Override
    public boolean isFinished(){

        return true;
    }

}
