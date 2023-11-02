package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.util.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.util.ProfileConstraints;
import org.firstinspires.ftc.teamcode.util.ProfileState;

@Config
public class ArmSubsystem extends SubsystemBase {

    ServoImplEx linkageServo;
    ServoImplEx armServo;

    ServoImplEx armRightServo;

    ServoImplEx rotateServo;

    ServoImplEx transferServo;

    ServoImplEx hornServo;

    AbsoluteAnalogEncoder transferEncoder;
    AbsoluteAnalogEncoder armEncoder;


    private RobotStateSubsystem robotState;

    //this is the arm rotation mount
    public static double MIDDLE_ROTATE_POS = 0.475; //higher moves to the left

    public static double ARM_SERVO_HOME = 0.1;

    public static double TRANSFER_ROTATE = 0.5;

    public ArmSubsystem(HardwareMap map, RobotStateSubsystem state){
        linkageServo = map.get(ServoImplEx.class, "linkage");
        armServo = map.get(ServoImplEx.class, "arm");
        armRightServo = map.get(ServoImplEx.class, "armRight");

        //rotation servo is under funky block
        rotateServo = map.get(ServoImplEx.class, "rotate");

        //this has the transfer connected to it.
        transferServo = map.get(ServoImplEx.class, "transfer");

        //this servo is the servo horn
        hornServo = map.get(ServoImplEx.class, "horn");

        //set the motor directions of the servos
        linkageServo.setDirection(Servo.Direction.REVERSE);
        armServo.setDirection(Servo.Direction.REVERSE);

        //not sure if axon max's need a custom PWM range - can't hurt to set
        //armRightServo.setPwmRange(new PwmControl.PwmRange(505, 2495));
        //armServo.setPwmRange(new PwmControl.PwmRange(505, 2495));

        rotateServo.setPosition(MIDDLE_ROTATE_POS);
        hornServo.setPosition(0);
        transferServo.setPosition(TRANSFER_ROTATE);
        linkageServo.setPosition(0);



        //Be sure to set the reverse direction before calling the set position method.
        armRightServo.setDirection(Servo.Direction.REVERSE);
        armRightServo.setPosition(ARM_SERVO_HOME);
        armServo.setPosition(ARM_SERVO_HOME);

        robotState = state;


    }

    public void loop(){



    }

    public double getTransferAInput(){

        return transferEncoder.getCurrentPosition();
    }

    public double getArmAngle(){
        return armEncoder.getCurrentPosition();
    }

    public void LockLinkage(){
        //lock the transfer into the intake
        linkageServo.setPosition(0.3);
    }

    public void UnlockLinkage(){
        linkageServo.setPosition(0);
    }


    public void HomeArm(){
        armServo.setPosition(0);
    }

    public void ArmUp(){
        //move the arm upright
        //go a little over the upright position to align with the backdrop
        armServo.setPosition(0.88);
        armRightServo.setPosition(0.88);
    }

    public boolean IsArmUp(){
        //TODO: is the arm in the up position
        return true;
    }

    public void ArmAngle(double angle){
        armServo.setPosition(angle);
    }

    public void Rotate(double angle){
        //need to work out a system, i.e. 90 degrees is middle, < 90 is left, > 90 is right
    }

    public void TransferAngle(double angle){
        transferServo.setPosition(angle);
    }

    public void HomeTransfer(){

        //returns the arm to the home position

        armServo.setPosition(ARM_SERVO_HOME);
        armRightServo.setPosition(ARM_SERVO_HOME);
    }

    public void HomeTransferSlow(){
        //this tries to avoid the massive slam down of the arm
        armServo.setPosition(ARM_SERVO_HOME + 0.2);
        armRightServo.setPosition(ARM_SERVO_HOME + 0.2);
    }

    public boolean IsArmHome(){

        return true;

    }

    public void ArmUpInMiddle(){
        //Arm moving up, in the middle position
        armServo.setPosition(MIDDLE_ROTATE_POS);
        armRightServo.setPosition(MIDDLE_ROTATE_POS);
    }
    public boolean IsArmUpInMiddle(){

        return true;
    }

    public void ArmLeftPosition(){
        //TODO: find the left position of the servo
        rotateServo.setPosition(0);
    }

    public void ArmCenterPosition(){
        //make sure the center position is in the middle of the arm
        //ready for the arm to go down
        rotateServo.setPosition(MIDDLE_ROTATE_POS);
    }

    public boolean IsArmLeft(){
        //TODO: find the left position of the servo
        return true;
    }

    public void ArmRightPosition(){
        //TODO: find the right position of the servo
        rotateServo.setPosition(0.7);
    }

    public boolean IsArmRight(){
        //TODO: find encoder position of the right arm servo.
        return true;
    }

    public void ExtendoIn(){
        linkageServo.setPosition(0);
    }

    public void ExtendoOut(){

        linkageServo.setPosition(1);
    }
    public void LockExtendo(){
        linkageServo.setPosition(0.1);
    }

    public void DropPixel(){
        hornServo.setPosition(0.1);
    }

    public void LockPixel(){

        hornServo.setPosition(0);
    }

    @Override
    public void periodic() {
        super.periodic();
    }


}
