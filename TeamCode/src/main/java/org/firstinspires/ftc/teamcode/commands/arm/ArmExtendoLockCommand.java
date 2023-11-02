package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class ArmExtendoLockCommand extends CommandBase {

    private ArmSubsystem armSubsystem;

    public ArmExtendoLockCommand(ArmSubsystem arm) {
        armSubsystem = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {

        armSubsystem.LockExtendo();
    }

    @Override
    public boolean isFinished(){

        return true;
    }
}