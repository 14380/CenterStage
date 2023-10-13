package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeOnCommand extends CommandBase {

    private IntakeSubsystem intakeSubsystem;

    public IntakeOnCommand(IntakeSubsystem intake){
        this.intakeSubsystem = intake;
    }

    @Override
    public void initialize(){
        this.intakeSubsystem.IntakeOn();
    }
    @Override
    public boolean isFinished(){
        return true;
    }

}
