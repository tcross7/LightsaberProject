// includes
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SerialFlash.h>
#include <Adafruit_NeoPixel.h>
#include <NXPMotionSense.h>

// Audio setup
AudioPlaySerialflashRaw  playFlashRaw1;  //xy=149,388
AudioPlaySerialflashRaw  playFlashRaw2;  //xy=149,388
AudioPlaySerialflashRaw  playSwingRaw;  //xy=149,388
AudioMixer4              mixer1;         //xy=445,386
AudioOutputAnalog        dac1;           //xy=591,379
AudioConnection          patchCord1(playFlashRaw1, 0, mixer1, 0);
AudioConnection          patchCord2(playFlashRaw2, 0, mixer1, 1);
AudioConnection          patchCord3(playSwingRaw, 0, mixer1, 2);
AudioConnection          patchCord4(mixer1, dac1);

// defines
#define PROP_AMP_ENABLE     5
#define FLASH_CHIP_SELECT  6
#define PIN_PROP_LED_ENABLE  7      // Pin to enable 11/13 as outputs
#define PIN_PROP_LED_DATA 11      // Pin #1 for 5v LEDs (data for APA102/dotstar leds) 
#define PIN_PROP_LED_CLK  13      // Pin #2 for 5v LEDs (clock for APA102/dotstar leds) 
#define PIN_NEOPIXEL    PIN_PROP_LED_DATA // PIN_PROP_LED_DATA to use prop shield level shifter
#define NUM_LEDS    116  // dot 0-143

// Color definitions
#define CYAN 0x0FFBFF
#define BLUE 0x00FF00
#define GREEN 0x0000FF 
#define RED 0xFF0000
#define PURPLE 0x950B95 //0xCA32CA //0xE111D5
#define YELLOW 0xFFFF33//0xF7FA3F
#define ORANGE 0xFF8000//0xEF861D //0xFC9015
#define BLACK 0x000000
#define WHITE 0xFFFFFF

// color list
int colors[] = {CYAN, BLUE, GREEN, RED, PURPLE, YELLOW, ORANGE};
int altcolors[] = {WHITE, CYAN, WHITE, WHITE, 0xFF99FF, WHITE, RED};
int list_progress = 0;
int current_color = colors[0];
int current_altcolor = altcolors[0];

// variables
int BLADE_LOC;
int CRASH_TIME;
int state;
int off = 0;
int poweron = 1;
int on = 2;
int clash = 3;
int deflect = 4;
int swing = 5;
int poweroff = 6;
int press_count = 0;
int swingSounds = 16;
int lastSwingSound = swingSounds;
int motionThreshold = 2;
float lastPitch = 0, lastRoll = 0, lastHeading = 0;

// accelerometer setup
NXPMotionSense imu;
NXPSensorFusion filter;

// LED setup
Adafruit_NeoPixel strip(115, PIN_PROP_LED_DATA, NEO_GRB + NEO_KHZ800);
SPISettings neopixel_spi (20000000, MSBFIRST, SPI_MODE0);

void setup() {
    // wait up to 3 seconds for the Serial device to become available
    long unsigned debug_start = millis();
    while (!Serial && ((millis () - debug_start) <= 3000));
    
    Serial.begin(9600);
    
    // Enable the neopixels
    if (PIN_NEOPIXEL == PIN_PROP_LED_DATA || PIN_NEOPIXEL == PIN_PROP_LED_CLK){
        SPI.begin();
        pinMode(PIN_PROP_LED_ENABLE, OUTPUT);
        digitalWrite(PIN_PROP_LED_ENABLE, LOW);
    }
    strip.begin();
    strip.setBrightness(100);  
    
    // Enable the amplifier on the prop shield
    pinMode(PROP_AMP_ENABLE, OUTPUT);
    digitalWrite(PROP_AMP_ENABLE, HIGH);
    AudioMemory(80);
    
    // Set initial volume
    mixer1.gain (0, 0.6); // clean hum
    mixer1.gain (1, 0.4f); // distorted hum
    mixer1.gain (2, 0.7); // swings 

    // Initialize pins for buttons
    pinMode(2, INPUT_PULLUP);
    pinMode(22, INPUT_PULLUP);
    digitalWrite(2, HIGH);
    digitalWrite(22, HIGH);
    
    // Start SerialFlash
    if (!SerialFlash.begin(FLASH_CHIP_SELECT)) {
        while (1){
            Serial.println("Cannot access SPI Flash chip\n");
            delay(1000);
        }
    }

    // Start motion sensor
    imu.begin();
    filter.begin(100);

    // start state machine in off state
    state = off;
}

void loop() {  
    switch (state) {
        case 0:
            state_off();
        break;
        case 1:
            state_poweron();
        break;
        case 2:
            state_on();
        break;
        case 3:
            state_clash();
        break;
        case 4:
            state_deflect();
        break;
        case 5:
            state_swing();
        break;
        case 6:
            state_poweroff();
        break;
    }
}

void detectMotion() {
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float roll, pitch, heading;
    
    if (imu.available()) {
        // Read the motion sensors
        imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);
        // Update the SensorFusion filter
        filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
        // print the heading, pitch and roll
        roll = filter.getRoll();
        pitch = filter.getPitch();
        heading = filter.getYaw();
        // does it need a time threshold too?
        float headingDiff = abs(lastHeading - heading);
        float pitchDiff = abs(lastPitch - pitch);

        if(lastHeading != 0){
            if(pitchDiff > motionThreshold || headingDiff > motionThreshold){      
                Serial.println("Swing detected!!");
                //cyle through swing sounds
                lastSwingSound++;
                if(lastSwingSound>swingSounds){
                    lastSwingSound=1;
                }
                String swingFile = "cswingX.raw";
                swingFile.replace("X",lastSwingSound);
                char charBuf[50];
                swingFile.toCharArray(charBuf, 50); 
                triggerSwing(charBuf);  // needs sequence to iterate through
            }
        }
        lastHeading = heading;
        lastPitch = pitch;
    }
}

void triggerSwing(const char *filename){
    if(playSwingRaw.isPlaying() == 0){
        Serial.println(filename);
        delay(5);
        playSwingRaw.play(filename);
    } else {
        Serial.println("already swinging");
    }
}

void LS_extend(){
    // Enable LEDs and put the pin back into digital mode
    if (PIN_NEOPIXEL == PIN_PROP_LED_DATA || PIN_NEOPIXEL == PIN_PROP_LED_CLK){
        SPI.beginTransaction(neopixel_spi);
        digitalWrite(PIN_PROP_LED_ENABLE, HIGH);
        pinMode(PIN_NEOPIXEL, OUTPUT);
    }
    // SET COLOR
    for (int i = 0; i < NUM_LEDS; i = i+3){
        strip.setPixelColor(i, current_color);
        strip.setPixelColor(i+1, current_color);
        strip.setPixelColor(i+2, current_color);
        strip.setPixelColor(i+3, current_color);

        strip.setPixelColor((NUM_LEDS - (i/2)), current_color);
        strip.setPixelColor((NUM_LEDS - (i/2))+1, current_color);
        strip.setPixelColor((NUM_LEDS - (i/2))+2, current_color);
        strip.setPixelColor((NUM_LEDS - (i/2))+3, current_color); 
        strip.show();
        //Serial.println(strip.numPixels());
        delay(1);
    }
    // Return pins 11/13 to be in SPI mode and disable the LEDs
    if (PIN_NEOPIXEL == PIN_PROP_LED_DATA || PIN_NEOPIXEL == PIN_PROP_LED_CLK){
        volatile uint32_t *reg;
        SPI.endTransaction();
        digitalWrite(PIN_PROP_LED_ENABLE, LOW);
        reg = portConfigRegister(PIN_NEOPIXEL);
        *reg = PORT_PCR_MUX(2);
    }
    playFlashRaw1.play("cpoweron.raw"); // Start playing the file.  This sketch continues to run while the file plays.  8/8/20 - moved from flashraw1 to swingraw
    delay(5); // A brief delay for the library read RAW info
    Serial.println(playSwingRaw.isPlaying());
    while (playFlashRaw1.isPlaying()) { // do nothing
        ;
    }  
}

void LS_clash(){
    // Enable LEDs and put the pin back into digital mode
    if (PIN_NEOPIXEL == PIN_PROP_LED_DATA || PIN_NEOPIXEL == PIN_PROP_LED_CLK){
        SPI.beginTransaction(neopixel_spi);
        digitalWrite(PIN_PROP_LED_ENABLE, HIGH);
        pinMode(PIN_NEOPIXEL, OUTPUT);
    }
    // Do effect
    BLADE_LOC = random(0, 57); // creates random location for blade clash
    for(int time = 0; time < 8; time++) { // replace with button press duration later?   
        CRASH_TIME = random(7, 10); // creates random time delay for blade clash
        strip.setPixelColor(BLADE_LOC, WHITE);
        strip.setPixelColor(BLADE_LOC+1, WHITE);
        strip.setPixelColor(BLADE_LOC-1, WHITE);
        strip.setPixelColor(BLADE_LOC+2, WHITE);
        strip.setPixelColor(BLADE_LOC-2, WHITE);
        strip.setPixelColor((NUM_LEDS - BLADE_LOC), WHITE);
        strip.setPixelColor((NUM_LEDS - BLADE_LOC)+1, WHITE);
        strip.setPixelColor((NUM_LEDS - BLADE_LOC)-1, WHITE);
        strip.setPixelColor((NUM_LEDS - BLADE_LOC)+2, WHITE);
        strip.setPixelColor((NUM_LEDS - BLADE_LOC)-2, WHITE);
        strip.show();
        delay(CRASH_TIME); // wait a random amount of time so it feels real
        
        strip.setPixelColor(BLADE_LOC, current_color);
        strip.setPixelColor(BLADE_LOC+1, current_color);
        strip.setPixelColor(BLADE_LOC-1, current_color);
        strip.setPixelColor(BLADE_LOC+2, current_color);
        strip.setPixelColor(BLADE_LOC-2, current_color);
        strip.setPixelColor((NUM_LEDS - BLADE_LOC), current_color);
        strip.setPixelColor((NUM_LEDS - BLADE_LOC)+1, current_color);
        strip.setPixelColor((NUM_LEDS - BLADE_LOC)-1, current_color);
        strip.setPixelColor((NUM_LEDS - BLADE_LOC)+2, current_color);
        strip.setPixelColor((NUM_LEDS - BLADE_LOC)-2, current_color);
        strip.show();
        delay(CRASH_TIME);
    }
    // Return pins 11/13 to be in SPI mode and disable the LEDs
    if (PIN_NEOPIXEL == PIN_PROP_LED_DATA || PIN_NEOPIXEL == PIN_PROP_LED_CLK){
        volatile uint32_t *reg;
        SPI.endTransaction();
        digitalWrite(PIN_PROP_LED_ENABLE, LOW);
        reg = portConfigRegister(PIN_NEOPIXEL);
        *reg = PORT_PCR_MUX(2);
    }
    playFlashRaw1.play("cclash.raw"); // Start playing the file.  This sketch continues to run while the file plays.  8/8/20 - moved from flashraw1 to swingraw
    delay(5); // A brief delay for the library read RAW info
    while (playFlashRaw1.isPlaying()) { // do nothing
        ;
    }  
}

void LS_deflect(){
    // Enable LEDs and put the pin back into digital mode
    if (PIN_NEOPIXEL == PIN_PROP_LED_DATA || PIN_NEOPIXEL == PIN_PROP_LED_CLK){
        SPI.beginTransaction(neopixel_spi);
        digitalWrite(PIN_PROP_LED_ENABLE, HIGH);
        pinMode(PIN_NEOPIXEL, OUTPUT);
    }
    // Do effect
    BLADE_LOC = random(0, 57); // creates random location for blade clash
    for(int time = 0; time < 8; time++) { // replace with button press duration later?   
        CRASH_TIME = random(7, 10); // creates random time delay for blade clash
        strip.setPixelColor(BLADE_LOC, 255,0,80);
        strip.setPixelColor(BLADE_LOC+1, 255,0,80);
        strip.setPixelColor(BLADE_LOC-1, 255,0,80);
        strip.setPixelColor(BLADE_LOC+2, 255,0,80);
        strip.setPixelColor(BLADE_LOC-2, 255,0,80);
        strip.setPixelColor((NUM_LEDS - BLADE_LOC), 255,0,80);
        strip.setPixelColor((NUM_LEDS - BLADE_LOC)-2, 255,0,80);
        strip.setPixelColor((NUM_LEDS - BLADE_LOC)+1, 255,0,80);
        strip.setPixelColor((NUM_LEDS - BLADE_LOC)-1, 255,0,80);
        strip.setPixelColor((NUM_LEDS - BLADE_LOC)+2, 255,0,80);
       
        strip.show();
        delay(CRASH_TIME); // wait a random amount of time so it feels real
        
        strip.setPixelColor(BLADE_LOC, current_color);
        strip.setPixelColor(BLADE_LOC+1, current_color);
        strip.setPixelColor(BLADE_LOC-1, current_color);
        strip.setPixelColor(BLADE_LOC+2, current_color);
        strip.setPixelColor(BLADE_LOC-2, current_color);
        strip.setPixelColor((NUM_LEDS - BLADE_LOC), current_color);
        strip.setPixelColor((NUM_LEDS - BLADE_LOC)+1, current_color);
        strip.setPixelColor((NUM_LEDS - BLADE_LOC)-1, current_color);
        strip.setPixelColor((NUM_LEDS - BLADE_LOC)+2, current_color);
        strip.setPixelColor((NUM_LEDS - BLADE_LOC)-2, current_color);
        strip.show();
        delay(CRASH_TIME);
    }
    // Return pins 11/13 to be in SPI mode and disable the LEDs
    if (PIN_NEOPIXEL == PIN_PROP_LED_DATA || PIN_NEOPIXEL == PIN_PROP_LED_CLK){
        volatile uint32_t *reg;
        SPI.endTransaction();
        digitalWrite(PIN_PROP_LED_ENABLE, LOW);
        reg = portConfigRegister(PIN_NEOPIXEL);
        *reg = PORT_PCR_MUX(2);
    }
    playFlashRaw1.play("cdeflect.raw"); // Start playing the file.  This sketch continues to run while the file plays.  8/8/20 - moved from flashraw1 to swingraw
    delay(5); // A brief delay for the library read RAW info
    while (playFlashRaw1.isPlaying()) { // do nothing
        ;
    }  
}

void LS_retract(){
    // Enable LEDs and put the pin back into digital mode
    if (PIN_NEOPIXEL == PIN_PROP_LED_DATA || PIN_NEOPIXEL == PIN_PROP_LED_CLK){
        SPI.beginTransaction(neopixel_spi);
        digitalWrite(PIN_PROP_LED_ENABLE, HIGH);
        pinMode(PIN_NEOPIXEL, OUTPUT);
    }
    // SET COLOR
    for (int i = NUM_LEDS; i > 0; i=i-3){
        strip.setPixelColor((i/2), BLACK);
        strip.setPixelColor((i/2)+1, BLACK);
        strip.setPixelColor((i/2)+2, BLACK);
        strip.setPixelColor((i/2)+3, BLACK);
        strip.setPixelColor((NUM_LEDS - (i/2)), BLACK);
        strip.setPixelColor((NUM_LEDS - (i/2))+1, BLACK);
        strip.setPixelColor((NUM_LEDS - (i/2))+2, BLACK);
        strip.setPixelColor((NUM_LEDS - (i/2))+3, BLACK);
        strip.show();
        delayMicroseconds(50);
    }    
    // Return pins 11/13 to be in SPI mode and disable the LEDs
    if(PIN_NEOPIXEL == PIN_PROP_LED_DATA || PIN_NEOPIXEL == PIN_PROP_LED_CLK){
        volatile uint32_t *reg;
        SPI.endTransaction();
        digitalWrite(PIN_PROP_LED_ENABLE, LOW);
        reg = portConfigRegister(PIN_NEOPIXEL);
        *reg = PORT_PCR_MUX(2);
    }
    playFlashRaw1.play("cpoweroff.raw"); // Start playing the file.  This sketch continues to run while the file plays.  8/8/20 - moved from flashraw1 to swingraw
    delay(5); // A brief delay for the library read RAW info
    while (playFlashRaw1.isPlaying()) { // do nothing
        ;
    }
}

void peak(bool state){

    if (state == 1){
        // Enable LEDs and put the pin back into digital mode
        if (PIN_NEOPIXEL == PIN_PROP_LED_DATA || PIN_NEOPIXEL == PIN_PROP_LED_CLK){
            SPI.beginTransaction(neopixel_spi);
            digitalWrite(PIN_PROP_LED_ENABLE, HIGH);
            pinMode(PIN_NEOPIXEL, OUTPUT);
        }
        // Do effect
        strip.setPixelColor(0, current_color);         //  Set pixel's color (in RAM)
        strip.setPixelColor(115, current_color);         //  Set pixel's color (in RAM)
        strip.setPixelColor(116, current_color); 
        strip.show();
    }
    else if (state == 0){
        // Enable LEDs and put the pin back into digital mode
        if (PIN_NEOPIXEL == PIN_PROP_LED_DATA || PIN_NEOPIXEL == PIN_PROP_LED_CLK){
            SPI.beginTransaction(neopixel_spi);
            digitalWrite(PIN_PROP_LED_ENABLE, HIGH);
            pinMode(PIN_NEOPIXEL, OUTPUT);
        }
        // Do effect
        strip.setPixelColor(0, BLACK);         //  Set pixel's color (in RAM)
        strip.setPixelColor(115, BLACK);         //  Set pixel's color (in RAM)
        strip.setPixelColor(116, BLACK);
        strip.show();
        }
}

int button1_check(){
    if (digitalRead(22) == 0) { // register first push
        Serial.println("Effect button pressed");
        delay(250); // wait a quarter second
        if (digitalRead(22) == 0) { // if button is still pressed
            return 2; // 2 for long press
        }
        else {
            return 1; // 1 for single press
        } 
    }
    else {
        return 0; 
    }
}

int button2_check(){ 
    if (digitalRead(2) == 0) { 
        Serial.println("Power button pressed");
        return 1;
    }
    else {
        return 0;
    }
}

void button1_check_off(){
    peak(0); // turn off lights in case they weren't already
    
    if (digitalRead(22) == 0) { // every time button is pressed in off state
        unsigned long initTime = millis();
        while (unsigned long currentTime = millis() - initTime < 1200){ // for 1.5 sec ms
            pot_check_off(); // constantly update based on pot value
            peak(1); // turn lights on
        }
        peak(0);   // turn lights off
    }
}

void pot_check_off(){
    int pot_val = analogRead(A0);
    int n;    
    if (0 < pot_val && pot_val < 146){
        n = 0;
    }
    else if (146 < pot_val && pot_val < 292){
        n = 1;
    }
    else if (292 < pot_val && pot_val < 438){
        n = 2;
    }
    else if (438 < pot_val && pot_val < 584){
        n = 3;
    }
    else if (584 < pot_val && pot_val < 730){
        n = 4;
    }
    else if (730 < pot_val && pot_val < 876){
        n = 5;
    }
    else if (876 < pot_val && pot_val < 1023){
        n = 6;
    }
    current_color = colors[n];  
    current_altcolor = altcolors[n];
}

void state_off(){
    button1_check_off();
    int button2_state = button2_check(); 
    if ( button2_state == 1 ){
        state = poweron;
    }
    
}

void state_poweron(){
    Serial.println("power on\n"); 
    delay(500);
    LS_extend();
    state = on; 
}

void state_on(){        
    playFlashRaw1.play("chum.raw"); // Start playing the file.  This sketch continues to run while the file plays.
    playFlashRaw2.play("dhum.raw");  
    delay(5); // A brief delay for the library read RAW info
    Serial.println(playFlashRaw2.isPlaying());
    while (playFlashRaw2.isPlaying() ) { // Update buttons as the file is playing
        detectMotion(); // play function to detect motion and play swings
        
        int pot_gain = abs((analogRead(A0) - 23)/100); // 0 to 1023 down to about 0 to 1
        mixer1.gain(0, pot_gain*0.6); // clean
        mixer1.gain(1, pot_gain*0.4); // distorted
        mixer1.gain(2, pot_gain*0.7); // swing
         
        int button1_state = button1_check();
        int button2_state = button2_check();
        if ( button2_state == 1 ){ // if the power button is pressed
            state = poweroff;
            playFlashRaw2.stop();
            break; // break out of the while loop if a button is pressed
        }
        else if ( button1_state == 1 ){ 
            state = deflect;
            break;
        }
        else if ( button1_state == 2 ){
            state = clash;
            break;
        }
    }
}

void state_clash(){
    Serial.println("clash"); 
    LS_clash();
    state = on; 
}

void state_deflect(){
    Serial.println("deflect");
    LS_deflect();
    state = on;   
}

void state_swing(){
    Serial.println("swing\n");
    delay(750);
    Serial.println("pressing off button...\n");
    delay(750);
    state = poweroff;  
}

void state_poweroff(){
    Serial.println("power off\n"); 
    LS_retract();
    state = off; 
}
