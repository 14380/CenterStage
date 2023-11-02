package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class MiddleArmCenterCommand extends CommandBase {

    private ArmSubsystem armSubsystem;

    public MiddleArmCenterCommand(ArmSubsystem arm){
        armSubsystem = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        armSubsystem.ArmUpInMiddle();
    }

    @Override
    public boolean isFinished(){

        return true;
    }
}
