package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class UnlockTransferCommand extends CommandBase {

    private ArmSubsystem armSubsystem;

    public UnlockTransferCommand(ArmSubsystem arm){
        armSubsystem = arm;

    }

    @Override
    public void initialize() {
        armSubsystem.UnlockLinkage();
    }
}
