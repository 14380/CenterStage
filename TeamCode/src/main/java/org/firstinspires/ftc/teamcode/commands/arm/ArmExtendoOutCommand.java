package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class ArmExtendoOutCommand extends CommandBase {

    private ArmSubsystem armSubsystem;

    public ArmExtendoOutCommand(ArmSubsystem arm){
        armSubsystem = arm;

    }

    @Override
    public void initialize() {
        armSubsystem.ExtendoOut();
    }

    @Override
    public boolean isFinished(){

        return true;
    }
}
