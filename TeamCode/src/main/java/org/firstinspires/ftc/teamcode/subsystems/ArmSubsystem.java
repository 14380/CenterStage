package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.util.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.util.ProfileConstraints;
import org.firstinspires.ftc.teamcode.util.ProfileState;

@Config
public class ArmSubsystem extends SubsystemBase {

    ServoImplEx linkageServo;
    CRServoImplEx armServo;
    ServoImplEx rotateServo;

    CRServoImplEx transferServo;

    public static PIDController transferPID;

    public static PIDController armPID;

    public static double transferP = 0.4;
    public static double transferI = 0.00002;
    public static double transferD = 0.009;

    public static double armP = 0.4;
    public static double armI = 0.00002;
    public static double armD = 0.009;

    double transferTargetPos = 2.55;
    double armTargetPos = 1.8;


    AbsoluteAnalogEncoder transferEncoder;
    AbsoluteAnalogEncoder armEncoder;


    private AsymmetricMotionProfile profile;
    private ProfileConstraints constraints;
    private ProfileState state;
    private PIDController controller;
    public ElapsedTime timer;

    private double position = 0.0;
    private double targetPosition = 0.0;
    private double power = 0.0;
    private double tolerance = 0.0;
    private double feedforwardMin = 0.0;
    private double feedforwardMax = 0.0;
    private double currentFeedforward = 0.0;
    private double targetPositionOffset = 0.0;
    private double offset = 0.0;

    private boolean reached = false;

    private RobotStateSubsystem robotState;


    public ArmSubsystem(HardwareMap map, RobotStateSubsystem state){
        linkageServo = map.get(ServoImplEx.class, "linkage");
        armServo = map.get(CRServoImplEx.class, "arm");
        rotateServo = map.get(ServoImplEx.class, "rotate");
        transferServo = map.get(CRServoImplEx.class, "transfer");

        transferPID = new PIDController(transferP,transferI,transferD);
        armPID = new PIDController(armP, armI, armD);

        AnalogInput aiTransfer = map.get(AnalogInput.class, "transferEncoder");
        transferEncoder = new AbsoluteAnalogEncoder(aiTransfer);

        //the arm needs gravity feedforward
        AnalogInput aiArm = map.get(AnalogInput.class, "armEncoder");
        armEncoder = new AbsoluteAnalogEncoder(aiArm);

        rotateServo.setPosition(0);

        armEncoder.zero(2.4);
        armEncoder.setWraparound(true);

        robotState = state;

    }

    public void loop(){

        if (timer == null) {
            timer = new ElapsedTime();
        }

        if (profile != null) {
            this.state = profile.calculate(timer.time());
            this.targetPosition = state.x + targetPositionOffset;
        }

        double transferArmPos = transferEncoder.getCurrentPosition();
        double transferPower = transferPID.calculate(transferArmPos, transferTargetPos);
        transferPID.setPID(transferP, transferI, transferD);
        transferServo.setPower(transferPower);

        double armPos = armEncoder.getCurrentPosition();
       // double armPower = armPID.calculate(armPos, armTargetPos);
        armPID.setPID(armP, armI, armD);

        position = armPos;
        targetPosition = armTargetPos;

        this.power = armPID.calculate(position, targetPosition + targetPositionOffset);

        this.power += currentFeedforward * Math.signum((targetPosition + targetPositionOffset) - position);

        this.power = MathUtils.clamp(power, -1, 1);

        this.reached = Math.abs((targetPosition + targetPositionOffset) - position) < tolerance;

        //arm needs gravity feedforward
       // double ticks_per_rev = 6;
       // double ff = Math.cos(Math.toRadians( armTargetPos / ticks_per_rev)) * 0.109;
        armServo.setPower(power);



    }

    public double getTransferAInput(){

        return transferEncoder.getCurrentPosition();
    }

    public double getArmAngle(){
        return armEncoder.getCurrentPosition();
    }

    public void LockLinkage(){
        //TODO: lock linkage into intake

    }

    public void UnlockLinkage(){
        //TODO: Find length for unlocking linkage
    }


    public void HomeArm(){

    }

    public void ArmUp(){
        //TODO move the arm upright
    }

    public boolean IsArmUp(){
        //TODO: is the arm in the up position
        return true;
    }

    public void ArmAngle(double angle){

        armTargetPos = angle;
    }

    public void Rotate(double angle){
        //need to work out a system, i.e. 90 degrees is middle, < 90 is left, > 90 is right
    }

    public void TransferAngle(double angle){

        transferTargetPos = angle;
    }

    public void HomeTransfer(){

        //TODO: set the target position for home
    }

    public boolean IsArmHome(){

        //TODO: check the encoder position to determine if we are home
        return true;

    }

    public void ArmUpInMiddle(){
        //TODO: find the position to have the arm up in the middle.
    }
    public boolean IsArmUpInMiddle(){
        //TODO: check to see if the arm is in the up center positions
        return true;
    }

    public void ArmLeftPosition(){
        //TODO: find the left position of the servo
    }

    public boolean IsArmLeft(){
        //TODO: find the left position of the servo
        return true;
    }

    public void ArmRightPosition(){
        //TODO: find the right position of the servo
    }

    public boolean IsArmRight(){
        //TODO: find encoder position of the right arm servo.
        return true;
    }

    public void ExtendoIn(){

    }

    public void ExtendoOut(){

    }

    public void DropPixel(){

    }

    public void LockPixel(){

    }

    @Override
    public void periodic() {
        super.periodic();
    }


}
