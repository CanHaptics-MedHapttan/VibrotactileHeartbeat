/**
 **********************************************************************************************************************
 * @file       HapticHeartbeatExploration.pde
 * @author     Noami Catwell
 * @version    V1.0.0
 * @date       01-March-2024
 * @brief      PID Heartbeat
 */
 
/* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
import controlP5.*;
import java.lang.Math;

import java.io.File;
import java.io.IOException;
import java.io.*;
import javax.sound.sampled.AudioFormat.Encoding;
import javax.sound.sampled.UnsupportedAudioFileException;
import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioSystem;

import javax.sound.sampled.*;
/* end library imports *************************************************************************************************/  


/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 

ControlP5 cp5;

/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 


/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerMeter                      = 4000.0;
float             radsPerDegree                       = 0.01745;

/* pantagraph link parameters in meters */
float             l                                   = 0.07;
float             L                                   = 0.09;


/* end effector radius in meters */
float             rEE                                 = 0.006;


/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);
PVector           oldangles                              = new PVector(0, 0);
PVector           diff                              = new PVector(0, 0);


/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* device graphical position */
PVector           deviceOrigin                        = new PVector(0, 0);

/* World boundaries reference */
final int         worldPixelWidth                     = 1000;
final int         worldPixelHeight                    = 650;

float x_m,y_m;

// used to compute the time difference between two loops for differentiation
long oldtime = 0;
// for changing update rate
int iter = 0;

/// PID stuff

float P = 0.0;
// for I
float I = 0;
float cumerrorx = 0;
float cumerrory = 0;
// for D
float oldex = 0.0f;
float oldey = 0.0f;
float D = 0;

//for exponential filter on differentiation
float diffx = 0;
float diffy = 0;
float buffx = 0;
float buffy = 0;
float smoothing = 0.80;

float xr = 0;
float yr = 0;

// checking everything run in less than 1ms
long timetaken= 0;

// set loop time in usec (note from Antoine, 500 is about the limit of my computer max CPU usage)
int looptime = 500;
int heartRate = 5;

/* graphical elements */
PShape pGraph, joint, endEffector;
PShape wall;
PFont f;
/* end elements definition *********************************************************************************************/ 

int[] waveformAmplitudes;
float[] waveformValues;
boolean readWaveForm = false;
int waveIndex = 0;
/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 700);

  try{

    String filePath = "C:\\Users\\naomi\\Documents\\GIT\\ETS\\CanHaptics\\Project\\HapticHeartbeatExploration\\audio\\heartbeat_regular.wav";
    File audioFile = new File(filePath);
    waveformAmplitudes = getWavAmplitudes(audioFile);
    waveformValues = processAmplitudes(waveformAmplitudes);
    readWaveForm = true;
  }
  catch(UnsupportedAudioFileException | IOException ex){
    ex.printStackTrace();

  }
  
  /* GUI setup */
  smooth();

  cp5 = new ControlP5(this);
    
  cp5.addTextlabel("Prop")
      .setText("Gain for P(roportional)")
      .setPosition(0,250)
      .setColorValue(color(255,0,0))
      .setFont(createFont("Georgia",20))
      ;  
    cp5.addSlider("P") 
      .setPosition(0,275)
      .setSize(200,20)
      .setRange(0,2)
      .setValue(1)
      ;

    cp5.addTextlabel("Frame delay")
      .setText("Heart rate")
      .setPosition(0,375)
      .setColorValue(color(255,0,0))
      .setFont(createFont("Georgia",20))
      ;  
    cp5.addSlider("heartRate") 
      .setPosition(10,400)
      .setSize(200,20)
      .setRange(1,20)
      .setValue(5)
      ;
    cp5.addTextlabel("Loop time")
      .setText("Loop time")
      .setPosition(0,420)
      .setColorValue(color(255,0,0))
      .setFont(createFont("Georgia",20))
      ;  
    cp5.addSlider("looptime")
      .setPosition(10,450)
      .setSize(200,20)
      .setRange(250,4000)
      .setValue(500)
      .setNumberOfTickMarks(16)
      .setSliderMode(Slider.FLEXIBLE)
      ;


  /* device setup */
  
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */ 
  haplyBoard          = new Board(this, "COM6", 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  widgetOne.set_mechanism(pantograph);
  
  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
 
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  widgetOne.device_set_parameters();    
  
  /* visual elements setup */
  background(0);
  deviceOrigin.add(worldPixelWidth/2, 0);
  
  /* create pantagraph graphics */
  create_pantagraph();
  
  xr = 0;
  yr = 0;

  /* setup framerate speed */
  frameRate(baseFrameRate);
  f = createFont("Arial",16,true); // STEP 2 Create Font
  
  /* setup simulation thread to run at 1kHz */ 
  thread("SimulationThread");
}
/* end setup section ***************************************************************************************************/

/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(renderingForce == false){
    background(255); 
    update_animation(angles.x*radsPerDegree, angles.y*radsPerDegree, posEE.x, posEE.y);    
  }
}
/* end draw section ****************************************************************************************************/

int previousFrame = 0;
int currentFrame = 0;
float tick = 0;
float heartbeatValue = 0;

void AdvanceECG(){
  tick += 0.01; 
  heartbeatValue = (sin(((tick)*4)) + sin(tick*16)/4) * 3 * (-(floor(sin(tick * 2)) + 0.1)) * (1 - floor(sin(tick/1.5 % 2))); 
}


int noforce = 0;
long timetook = 0;
long looptiming = 0;

/* simulation section **************************************************************************************************/
public void SimulationThread(){
while(1==1) {
    long starttime = System.nanoTime();
    long timesincelastloop=starttime-timetaken;
    iter+= 1;
    // we check the loop is running at the desired speed (with 10% tolerance)
    if(timesincelastloop >= looptime*1000*1.1) {
      float freq = 1.0/timesincelastloop*1000000.0;      
    }
    else if(iter >= 1000) {
      float freq = 1000.0/(starttime-looptiming)*1000000.0;
       iter=0;
       looptiming=starttime;
    }
    
    currentFrame++;
    if(currentFrame - previousFrame > 20/heartRate){
      previousFrame = currentFrame;      
      AdvanceECG(); 

    }

    timetaken=starttime;
    
    renderingForce = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
      
      noforce = 0;
      angles.set(widgetOne.get_device_angles());
    
      posEE.set(widgetOne.get_device_position(angles.array()));

      posEE.set(device_to_graphics(posEE)); 
      x_m = xr*300; 
      y_m = yr*300+350;
      
      // Torques from difference in endeffector and setpoint, set gain, calculate force
      float xE = pixelsPerMeter * posEE.x;
      float yE = pixelsPerMeter * posEE.y;
      long timedif = System.nanoTime()-oldtime;

      println(heartbeatValue);

      float dist_X = heartbeatValue * random(-1,1);
      cumerrorx += dist_X*timedif*0.000000001;

      float dist_Y = heartbeatValue * random(-1,1);
      cumerrory += dist_Y*timedif*0.000000001;

      if(timedif > 0) {
        buffx = (dist_X-oldex)/timedif*1000*1000;
        buffy = (dist_Y-oldey)/timedif*1000*1000;            

        diffx = smoothing*diffx + (1.0-smoothing)*buffx;
        diffy = smoothing*diffy + (1.0-smoothing)*buffy;
        oldex = dist_X;
        oldey = dist_Y;
        oldtime=System.nanoTime();
      }
      
      fEE.x = constrain(P*dist_X,-4,4) + constrain(I*cumerrorx,-4,4) + constrain(D*diffx,-8,8);      
      fEE.y = constrain(P*dist_Y,-4,4) + constrain(I*cumerrory,-4,4) + constrain(D*diffy,-8,8); 
      int renderWaveForm  = 1;
      if(noforce==1)
      {
        fEE.x=0.0;
        fEE.y=0.0;
      }
      else if(readWaveForm && renderWaveForm==1){
        fEE.x = waveformValues[waveIndex % (waveformValues.length-1)] * 2; // OR fEE.x += waveFormValue;
        fEE.y = 0.0; // OR nothing
        waveIndex++;
        //read values of wav file, try downsampling to 300-500 Hz (ie Audacity)      
      }
      widgetOne.set_device_torques(graphics_to_device(fEE).array());      
    }

    widgetOne.device_write_torques();
    
    renderingForce = false;
    long timetook=System.nanoTime()-timetaken;
    if(timetook >= 1000000) {
    }
    else {
      while(System.nanoTime()-starttime < looptime*1000) {
      //NOP
      }
    }    
  }
}

/* end simulation section **********************************************************************************************/


/* helper functions section, place helper functions here ***************************************************************/
void create_pantagraph(){
  float lAni = pixelsPerMeter * l;
  float LAni = pixelsPerMeter * L;
  float rEEAni = pixelsPerMeter * rEE;
  
  pGraph = createShape();
  pGraph.beginShape();
  pGraph.fill(255);
  pGraph.stroke(0);
  pGraph.strokeWeight(2);
  
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.endShape(CLOSE);
  
  joint = createShape(ELLIPSE, deviceOrigin.x, deviceOrigin.y, rEEAni, rEEAni);
  joint.setStroke(color(0));
  
  endEffector = createShape(ELLIPSE, deviceOrigin.x, deviceOrigin.y, 2*rEEAni, 2*rEEAni);
  endEffector.setStroke(color(0));
  strokeWeight(5);  
}


void update_animation(float th1, float th2, float xE, float yE){
  background(255);
  pushMatrix();
  float lAni = pixelsPerMeter * l;
  float LAni = pixelsPerMeter * L;
  
  xE = pixelsPerMeter * xE;
  yE = pixelsPerMeter * yE;
  
  th1 = 3.14 - th1;
  th2 = 3.14 - th2;
    
  pGraph.setVertex(1, deviceOrigin.x + lAni*cos(th1), deviceOrigin.y + lAni*sin(th1));
  pGraph.setVertex(3, deviceOrigin.x + lAni*cos(th2), deviceOrigin.y + lAni*sin(th2));
  pGraph.setVertex(2, deviceOrigin.x + xE, deviceOrigin.y + yE);
  
  shape(pGraph);
  shape(joint);
  float[] coord;
  
  
  translate(xE, yE);
  shape(endEffector);
  popMatrix();
  arrow(xE,yE,fEE.x,fEE.y);
  textFont(f,16);
  fill(0);

  x_m = xr*300+500;       
  y_m = yr*300+350;

  pushMatrix();
  translate(x_m, y_m);
  popMatrix();  
}


PVector device_to_graphics(PVector deviceFrame){
  return deviceFrame.set(-deviceFrame.x, deviceFrame.y);
}


PVector graphics_to_device(PVector graphicsFrame){
  return graphicsFrame.set(-graphicsFrame.x, graphicsFrame.y);
}

void arrow(float x1, float y1, float x2, float y2) {
  x2=x2*10.0;
  y2=y2*10.0;
  x1=x1+500;
  x2=-x2+x1;
  y2=y2+y1;

  line(x1, y1, x2, y2);
  pushMatrix();
  translate(x2, y2);
  float a = atan2(x1-x2, y2-y1);
  rotate(a);
  line(0, 0, -10, -10);
  line(0, 0, 10, -10);
  popMatrix();
} 

/* end helper functions section ****************************************************************************************/




 


/****  waveform calculation code *****/

private static final double WAVEFORM_HEIGHT_COEFFICIENT = 1.3; // This fits the waveform to the swing node height


private int[] getWavAmplitudes(File file) throws UnsupportedAudioFileException , IOException {
				System.out.println("Calculting WAV amplitudes");
				
				//Get Audio input stream
				try (AudioInputStream input = AudioSystem.getAudioInputStream(file)) {
					AudioFormat baseFormat = input.getFormat();
					
					//Encoding
					Encoding encoding = AudioFormat.Encoding.PCM_UNSIGNED;
					float sampleRate = baseFormat.getSampleRate();
					int numChannels = baseFormat.getChannels();
					
					AudioFormat decodedFormat = new AudioFormat(encoding, sampleRate, 16, numChannels, numChannels * 2, sampleRate, false);
					int available = input.available();
					
					//Get the PCM Decoded Audio Input Stream
					try (AudioInputStream pcmDecodedInput = AudioSystem.getAudioInputStream(decodedFormat, input)) {
						final int BUFFER_SIZE = 4096; //this is actually bytes
						
						//Create a buffer
						byte[] buffer = new byte[BUFFER_SIZE];
						
						//Now get the average to a smaller array
						int maximumArrayLength = 100000;
						int[] finalAmplitudes = new int[maximumArrayLength];
						int samplesPerPixel = available / maximumArrayLength;
						
						//Variables to calculate finalAmplitudes array
						int currentSampleCounter = 0;
						int arrayCellPosition = 0;
						float currentCellValue = 0.0f;
						
						//Variables for the loop
						int arrayCellValue = 0;
						
						//Read all the available data on chunks
						while (pcmDecodedInput.readNBytes(buffer, 0, BUFFER_SIZE) > 0)
							for (int i = 0; i < buffer.length - 1; i += 2) {
								
								//Calculate the value
								arrayCellValue = (int) ( ( ( ( ( buffer[i + 1] << 8 ) | buffer[i] & 0xff ) << 16 ) / 32767 ) * WAVEFORM_HEIGHT_COEFFICIENT );
								
								//Tricker
								if (currentSampleCounter != samplesPerPixel) {
									++currentSampleCounter;
									currentCellValue += Math.abs(arrayCellValue);
								} else {
									//Avoid ArrayIndexOutOfBoundsException
									if (arrayCellPosition != maximumArrayLength)
										finalAmplitudes[arrayCellPosition] = finalAmplitudes[arrayCellPosition + 1] = (int) currentCellValue / samplesPerPixel;
									
									//Fix the variables
									currentSampleCounter = 0;
									currentCellValue = 0;
									arrayCellPosition += 2;
								}
							}
						
						return finalAmplitudes;
					} catch (Exception ex) {
						ex.printStackTrace();
					}
				} catch (Exception ex) {
					ex.printStackTrace();
					
				}
				
				//You don't want this to reach here...
				return new int[1];
			}


      private float[] processAmplitudes(int[] sourcePcmData) {
				System.out.println("Processing WAV amplitudes");
				
				//The width of the resulting waveform panel
				int width = 2000;//waveVisualization.width;
				float[] waveData = new float[width];
				int samplesPerPixel = sourcePcmData.length / width;
				
				//Calculate
				float nValue;
				for (int w = 0; w < width; w++) {
					
					//For performance keep it here
					int c = w * samplesPerPixel;
					nValue = 0.0f;
					
					//Keep going
					for (int s = 0; s < samplesPerPixel; s++) {
						nValue += ( Math.abs(sourcePcmData[c + s]) / 65536.0f );
					}
					
					//Set WaveData
					waveData[w] = nValue / samplesPerPixel;
				}
				
				System.out.println("Finished Processing amplitudes");
				return waveData;
			}
