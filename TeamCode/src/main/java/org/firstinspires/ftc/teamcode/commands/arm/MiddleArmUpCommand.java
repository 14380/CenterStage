package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class MiddleArmUpCommand extends CommandBase {

    private ArmSubsystem armSubsystem;

    public MiddleArmUpCommand(ArmSubsystem arm){
        armSubsystem = arm;

    }

    @Override
    public void initialize() {

        //TODO: check robot state to ensure that we can do this.
        armSubsystem.ArmUp();
    }

    @Override
    public boolean isFinished(){

        return true;
    }
}
