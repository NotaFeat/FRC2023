package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class TeleoperatedMode implements IRobotMode {
    
    private XboxController xboxController;
    private IDrive drive;
    private IArm arm;

    private static final double LEFT_STICK_EXPONENT = 3.0;
    private static final double RIGHT_STICK_EXPONENT = 3.0;
    private static final double ROTATION_THRESHOLD = 0.3;
    // check values above

    public TeleoperatedMode(IDrive drive, IArm arm){ 
        xboxController = new XboxController(PortMap.USB.XBOXCONTROLLER);
        this.drive = drive;
        this.arm = arm;
    }

    @Override
    public void init(){
        drive.init();
        arm.init();
        drive.balanceOff();
    }

     @Override
     public void periodic() {

        if (xboxController.getBackButton()) {
            drive.resetGyro();
        }

        if (xboxController.getStartButton()) {
            arm.resetPosition();
        }

        double leftX = xboxController.getLeftX();
        double leftY = -xboxController.getLeftY();

        leftX = Math.pow(leftX, LEFT_STICK_EXPONENT);
        leftY = Math.pow(leftY, LEFT_STICK_EXPONENT);

        drive.driveManual(leftY, leftX);

        //Process Rotation Control
        double rightX = xboxController.getRightX();
        double rightY = -xboxController.getRightY();

        rightX = Math.pow(rightX, RIGHT_STICK_EXPONENT);
        rightY = Math.pow(rightY, RIGHT_STICK_EXPONENT);

        double angle = Math.atan2(rightX, rightY);

        // Think Pythagorean Theorem
        if(Math.sqrt(Math.pow(rightX, 2) + Math.pow(rightY, 2)) > ROTATION_THRESHOLD) {
            drive.rotateAbsolute(angle);
        }

        if(xboxController.getYButton()) {
            arm.grabberOpen();
        }

       

        if(xboxController.getRightBumper()) {
            arm.shoulderUp();
        }

        if(xboxController.getLeftBumper()) {
            arm.shoulderDown();
        }

        if(xboxController.getRightTriggerAxis() > 0) {
            arm.elbowUp();
        }

        if(xboxController.getLeftTriggerAxis() > 0) {
            arm.elbowDown();
        }

        
        
        if(xboxController.getXButton()) {
            arm.midCone();
        }
        if(xboxController.getPOV() == 0) {
            arm.shelf();
        }
        if(xboxController.getPOV() == 90) {
            arm.rest();
        }
        if(xboxController.getBButton()) {
            drive.balanceOff();
        }
        if(xboxController.getAButton()) {
            drive.balanceOn();
        }

     }
    
}