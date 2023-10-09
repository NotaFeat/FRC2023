package frc.robot;

import java.lang.Thread;


public class AutonomousMode implements IRobotMode {

    private IDrive drive;
    private IGyroscopeSensor gyroscope;
    private IArm arm;
    
    public AutonomousMode(IDrive drive, IGyroscopeSensor gyroscope, IArm arm) {
        this.drive = drive;
        this.gyroscope = gyroscope;
        this.arm = arm;
        }


    public void init() {
        drive.resetGyro();
        drive.driveDistance(20, -.1, 0, () -> autoBalance());
        //drive.driveDistance(60, -0.6, 0, () -> arm1());
        //drive.driveDistance(90, -.6, 0, null);
        //arm.autoGrabber(80, 0.2, true, () -> arm1());
        //drive.balanceOn();
        // drive.rotateRelative(180, null);
    }

    public void pushBack() {
        drive.driveDistance(20, 0.1, 0, () -> autoBalance());
    }

    public void autoBalance() {
        drive.driveDistance(60, -0.6, 0, () -> driveUp());
        //drive.driveDistance(100, -0.5, 0, null);
    }

    public void driveUp() {
        drive.balanceOn();
    }

    public void arm1() {
        //arm.autoShoulder(9, .001, true, () -> arm2());
       // arm.autoShoulder(2.8, 0.001, true, () -> arm2());
       drive.balanceOn();
    }

    public void arm2() {
        //arm.autoElbow(3, 0.001, true, () ->arm3());
        arm.autoElbow(7, 0.002, true, () -> arm3());
    }

    public void arm3() {
        //arm.autoShoulder(10, 0.001, true, ()->arm4());
        arm.autoGrabber(-5, 0.2, true, () -> arm4());
    }

    public void arm4() {
        //arm.autoElbow(8, 0.001, true, ()->arm5());
        arm.autoElbow(0, .002, true, () -> arm5());
    }

    public void arm5() {
        arm.autoGrabber(0, 0.2, true, () -> arm6());
        //arm.autoShoulder(0, 0.001, true,() -> arm6()); 
    }

    public void arm6() {
        //drive.driveDistance(8, -0.2, 0, null);
        drive.driveDistance(140, -0.3, 0, null);
    }

    public void arm7() {
        drive.rotateRelative(180, () -> strafe());
    }

    public void strafe() {
       // drive.driveSpeed(0, 0.3, 0, () -> arm8());
    }

    public void arm8() {
        // drive.driveDistance(14, 0.3, 0,null);
        arm.autoShoulder(0, 0.001, true, () -> arm9());
    }

    public void arm9() {
        arm.autoElbow(8.5, 0.002, true, () -> grabCube());
    }

    public void grabCube() {
        arm.autoGrabber(80, 0, true, null);
    }

    // public void balance() {
    //     drive.balanceOn();
    // }

    @Override
    public void periodic() {
        // nothing to do
    }
}

