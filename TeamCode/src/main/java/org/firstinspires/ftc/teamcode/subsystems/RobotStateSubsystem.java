package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

public class RobotStateSubsystem extends SubsystemBase {

    public enum MiddleArmState{
        DOWN,
        UP
    }

    public enum VerticalHeight{
        ARM,
        DOWN,
        UP,
        POS0,
        POS1,
        POS2
    }

    public enum HorizontalHeight{
        RETRACTED,
        EXTENDED
    }

    public enum LaunchStatus
    {
        NONE,
        PLANE,
        LEFT_ARM,
        RIGHT_ARM
    }

    public LaunchStatus launchStatus = LaunchStatus.NONE;
    public VerticalHeight verticalHeight = VerticalHeight.DOWN;
    public HorizontalHeight horizontalHeight = HorizontalHeight.RETRACTED;
    public MiddleArmState middleArm = MiddleArmState.DOWN;
}
