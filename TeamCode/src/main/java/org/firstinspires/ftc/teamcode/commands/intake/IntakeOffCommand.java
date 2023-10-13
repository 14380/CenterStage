package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeOffCommand extends CommandBase {

    private IntakeSubsystem intakeSubsystem;

    public IntakeOffCommand(IntakeSubsystem intake){
        this.intakeSubsystem = intake;
    }

    @Override
    public void initialize(){

        this.intakeSubsystem.IntakeOff();
    }
    @Override
    public boolean isFinished(){

        return true;
    }

}
