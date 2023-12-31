package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeAdvanceCommand extends CommandBase {

    private IntakeSubsystem intakeSubsystem;

    public IntakeAdvanceCommand(IntakeSubsystem intake){
        this.intakeSubsystem = intake;
    }

    @Override
    public void initialize(){

        //check the robot state - can we advance?
        if(!this.intakeSubsystem.isExtended()) {
            this.intakeSubsystem.IntakeAdvance();
        }
    }
    @Override
    public boolean isFinished(){
        return true;
    }

}
