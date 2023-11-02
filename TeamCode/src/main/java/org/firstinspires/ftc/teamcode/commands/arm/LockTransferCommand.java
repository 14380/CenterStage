package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class LockTransferCommand extends CommandBase {

    private ArmSubsystem armSubsystem;

    public LockTransferCommand(ArmSubsystem arm){
        armSubsystem = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        armSubsystem.LockLinkage();
    }

    @Override
    public boolean isFinished(){

        return true;
    }
}
