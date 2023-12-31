package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class MiddleArmDownCommand extends CommandBase {

    private ArmSubsystem armSubsystem;

    public MiddleArmDownCommand(ArmSubsystem arm){
        armSubsystem = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        armSubsystem.HomeTransfer();
    }

    @Override
    public boolean isFinished(){

        return true;
    }
}
