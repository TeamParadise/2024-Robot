package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.Timer;;


public class LedSubsystem extends  SubsystemBase{
    //creating some objects we need in future 
    private static LedSubsystem instance;

    //Instances might be nessesary in a way it might be conlficting with other classes in case we have many leds or none.   
    public static LedSubsystem getInstance() {
    if (instance == null) {
      instance = new LedSubsystem();
      }
    return instance;
    }
    public static final CommandXboxController joystick = new CommandXboxController(0); // My joystick
    // introduce boolean variables for status
    // is the note inside
    public Trigger nodeInside = RobotContainer.primerBeamTrigger;
    //is the robot in shooting stance 
    public boolean aprilTagLocationGoodQuestionMark = false;
    //if button pressed to go demo 
    public boolean demo = false; 
    //if low battery
    private boolean lowBatteryAlert = false; 

    private boolean stop = false; 

    //all these others will be updated later but name serves its purpose
    private boolean lastEnabledAuto = false;
    private double lastEnabledTime = 0.0;
    private boolean estopped = false;
     
    //creating the led objects and its updater 
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer; 
    private final Notifier notifier;  

    //creating variables used for lighting patterns or/and paramters for led object definition
     public int loopCycleCount = 0;
    private static final int minLoopCycleCount = 10;
    private static final int staticLength = 45;
    private static final int staticSectionLength = 15;
    private static final double breathDuration = 1.0;
    private static final int length = 60; 
    private static final double autoFadeTime = 2.5; 
    private static final double autoFadeMaxTime = 5.0; 
    private static final double waveSlowCycleLength = 25.0;
    private static final double waveSlowDuration = 3.0;
    private static final double waveExponent = 0.4;

    //constructing the led object (private because im calling instances in other classes)
    private LedSubsystem(){
        // defining our objects and assingning them ports 
        led = new AddressableLED(0);
        buffer = new AddressableLEDBuffer(length);
        led.setLength(buffer.getLength());
        //setting the update proccess thorugh a notifier (I understand it can help redcue complications through updating led seperate from other systems (correct me if I'm wrong)) (Also cracked lambda that i took from Mechanical Advantage)
        notifier = new Notifier(
            () -> {
              synchronized (this) {
                //shows we are initilized when seeing black and white
                breath(Section.STATIC_LOW, Color.kWhite, Color.kBlack, 0.25, System.currentTimeMillis() / 1000.0);
                led.setData(buffer);
                led.start();
              }
            });
            notifier.startPeriodic(0.02);
    }



    //creating the repeating loop of the leds, however I used synchronized as it involves notifier
    public synchronized void periodic() {
      //could call it as this maybe
      // if(joystick.povUp().getAsBoolean()) stop = true; 


      // Exit during initial cycles
      loopCycleCount += 1;
        if (loopCycleCount < minLoopCycleCount) {
          return;
        }

      if(stop){
        still(Section.FULL, Color.kWhite);
        System.out.println("....");
      } else {
        // Update estop state
        if (DriverStation.isEStopped()) {
          estopped = true;
        }

        // Stop loading notifier if running
        notifier.stop();

        // Select LED mode
        // still(Section.FULL, Color.kBlack); // Default to off
        if (estopped) {
            still(Section.FULL, Color.kRed);
        } else if (DriverStation.isDisabled()) {
          if(joystick.povUp().getAsBoolean()) stop = true; 
          if (lastEnabledAuto && Timer.getFPGATimestamp() - lastEnabledTime < autoFadeMaxTime) {
            // Auto fade
            still(1.0 - ((Timer.getFPGATimestamp() - lastEnabledTime) / autoFadeTime), Color.kGreen);

          } else if (lowBatteryAlert) {
            // Low battery
            still(Section.FULL, Color.kOrangeRed);
          } 
          // //test
          // else if(stop){
          // wave(Section.FULL, Color.kGreen, Color.kRed, waveSlowCycleLength, waveSlowDuration);  
          // System.out.println(",");
          // }
          else {
            // Default pattern
            wave(Section.FULL, Color.kWhite, Color.kRed, waveSlowCycleLength, waveSlowDuration);  
            }
          } else if (nodeInside.getAsBoolean()) {
            //node inside
          still(Section.FULL, Color.kGreen);
        } else if (aprilTagLocationGoodQuestionMark) {
            //ready to shoot
          still(Section.FULL, Color.kYellow);
        }else {
          // Demo mode background
          if (demo) {
            wave(Section.FULL, Color.kGold, Color.kDarkBlue, waveSlowCycleLength, waveSlowDuration);
          }

        }
      } 
    // Update LEDs
    led.setData(buffer);
  }




  //creates a wave pattern using math and using parameters such as section of led, cycle length, colors, and duration. More can be found below 
  private void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < section.end(); i++) {
      x += xDiffPerLed;
      if (i >= section.start()) {
        double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
        if (Double.isNaN(ratio)) {
          ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
        }
        if (Double.isNaN(ratio)) {
          ratio = 0.5;
        }
        double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
        double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
        double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
        buffer.setLED(i, new Color(red, green, blue));
      }
    }
  }




    // creates a solid color on spefic sections
    private void still(Section section, Color color) {
        if (color != null) {
          for (int i = section.start(); i < section.end(); i++) {
            buffer.setLED(i, color);
          }
        }
      }
    // same thing as above but depends on percent for lastEnabledAuto && Timer.getFPGATimestamp
    private void still(double percent, Color color) {
        for (int i = 0; i < MathUtil.clamp(length * percent, 0, length); i++) {
          buffer.setLED(i, color);
        }
      }




    //creates a breath function for the leds 
    private void breath(Section section, Color c1, Color c2, double duration, double timestamp) {
        double x = ((timestamp % breathDuration) / breathDuration) * 2.0 * Math.PI;
        double ratio = (Math.sin(x) + 1.0) / 2.0;
        double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
        double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
        double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
        still(section, new Color(red, green, blue));
      }




    // creates an enum to seperate our led to sections to create the patterns mentioned above (inspired by team 6328, mechanical advantage)
    private static enum Section {
        //establishes our parts
        STATIC,
        SHOULDER,
        FULL,
        STATIC_LOW,
        STATIC_MID,
        STATIC_HIGH;
        
        private int start() {
          switch (this) {
            case STATIC:
              return 0;
            case SHOULDER:
              return staticLength;
            case FULL:
              return 0;
            case STATIC_LOW:
              return 0;
            case STATIC_MID:
              return staticSectionLength;
            case STATIC_HIGH:
              return staticLength - staticSectionLength;
            default:
              return 0;
          }
        }
    
        private int end() {
          switch (this) {
            case STATIC:
              return staticLength;
            case SHOULDER:
              return length;
            case FULL:
              return length;
            case STATIC_LOW:
              return staticSectionLength;
            case STATIC_MID:
              return staticLength - staticSectionLength;
            case STATIC_HIGH:
              return staticLength;
            default:
              return length;
          }
        }
      }
      public void setLowBatteryAlert(boolean lowBatteryAlert) {
      this.lowBatteryAlert = lowBatteryAlert;
    }



    public void setStop(boolean stop) {
      this.stop = stop;
    }

    public boolean isLowBatteryAlert() {
      return lowBatteryAlert;
    }

    public boolean isStop() {
      return stop;
    }
}

// WHAT I USED AS REFERENCE/inspiration- https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/subsystems/leds/Leds.java#L254
