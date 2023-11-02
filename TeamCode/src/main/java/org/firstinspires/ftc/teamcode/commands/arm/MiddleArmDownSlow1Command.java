package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class MiddleArmDownSlow1Command extends CommandBase {

    private ArmSubsystem armSubsystem;

    public MiddleArmDownSlow1Command(ArmSubsystem arm){
        armSubsystem = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        armSubsystem.HomeTransferSlow();
    }

    @Override
    public boolean isFinished(){

        return true;
    }
}
