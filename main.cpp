#include "mbed.h"
#include "MBed_Adafruit_GPS.h"

//Prints an error message location if fatal error occurs.
#define MBED_CONF_PLATFORM_ERROR_FILENAME_CAPTURE_ENABLED 0

//Configures pins and serial port.
Serial pc(USBTX, USBRX);
InterruptIn PPS(A1, PullNone);
DigitalOut led(LED1);

//Configures GPS.
Serial * gps_Serial = new Serial(D1,D0); //serial object for use w/ GPS
Adafruit_GPS myGPS(gps_Serial); //object of Adafruit's GPS class
char c; //when read via Adafruit_GPS::read(), the class returns single character stored here

/* Defines the two queues used, one for events and one for printing to the screen */
EventQueue printfQueue;
EventQueue GPSQueue;

/* Defines the timer */
Timer t;
int64_t usTime1 = 0;

int int_GPStime=0;
time_t whattime;
bool UpdateTime = true;

/* Converts standard time into Epoch time. Could delete this if no longer needed.*/
time_t asUnixTime(int year, int mon, int mday, int hour, int min, int sec) {
    struct tm   t;
    t.tm_year = year - 1900;
    t.tm_mon =  mon - 1;        // convert to 0 based month
    t.tm_mday = mday;
    t.tm_hour = hour;
    t.tm_min = min;
    t.tm_sec = sec;
    t.tm_isdst = -1;            // Is Daylight saving time on? 1 = yes, 0 = no, -1 = unknown
 
    return mktime(&t);          // returns seconds elapsed since January 1, 1970 (begin of the Epoch)
}

/* Prints to the serial console */
//This runs in the low priority thread
void Print_Sensors() {
    //Next two lines print current uc and gps time, possibly disadvantageous to do this rather than save them in variables
    //when us timer is saved/reset
    pc.printf("%d ", time(NULL));
    pc.printf("%d ", asUnixTime(myGPS.year+2000, myGPS.month, myGPS.day, myGPS.hour, myGPS.minute, myGPS.seconds));
    pc.printf("%lld \r\n", usTime1);
}

//Collects and parses GPS data
//This runs in the high priority thread
void GPS_data() {
    //Logs the us timer right after the pps triggers and resets it.
    usTime1 = t.read_us();
    t.reset();
    
    do{
        c = myGPS.read();   //queries the GPS
        //if (c) { printf("%c", c); }
        //check if we recieved a new message from GPS, if so, attempt to parse it,
        if ( myGPS.newNMEAreceived() ) {
            if ( myGPS.parse(myGPS.lastNMEA()) ){
                int_GPStime=asUnixTime(myGPS.year+2000, myGPS.month, myGPS.day, myGPS.hour, myGPS.minute, myGPS.seconds);
                //Set the uc rtc after succesfully receiving first sentence.
                if (UpdateTime) {
                    set_time(int_GPStime);
                    UpdateTime = false;
                } 
                break;
            }
        }
    }while( !myGPS.newNMEAreceived() );
}

// main() runs in its own thread in the OS
int main()
{
    myGPS.begin(9600);
    

    myGPS.sendCommand(PMTK_AWAKE);
    pc.printf("Wake Up GPS...\r\n");
    wait(1);
    myGPS.sendCommand(PMTK_STANDBY);
    pc.printf("Entering GPS Standby...\r\n");
    wait(1);
    myGPS.sendCommand(PMTK_SET_BAUD_57600);
    pc.printf("Set GPS Baud Rate to 57600...\r\n");
    myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    pc.printf("Set RM Message Format Only...\r\n");
    myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
    pc.printf("Set 5Hz Message Update Rate...\r\n");
    myGPS.sendCommand(PGCMD_NOANTENNA);
    pc.printf("Turn Off Antenna Messages...\r\n");

    /* resets and starts the timer */
    t.reset();
    t.start();
    pc.printf("Timer Reset and Started...\r\n");

    //Prints headers for each measurement.
    pc.printf("\r\nuC GPS uS\r\n");
    
    // normal priority thread for other events
    Thread eventThread(osPriorityHigh);
    eventThread.start(callback(&GPSQueue, &EventQueue::dispatch_forever));
  
    // low priority thread for calling printf()
    Thread printfThread(osPriorityLow);
    printfThread.start(callback(&printfQueue, &EventQueue::dispatch_forever));

    // call read_sensors 1 every second, automatically defering to the eventThread
    //Ticker GPSTicker;
    //Ticker ReadTicker;

    //GPSTicker.attach(GPSQueue.event(&GPS_data), 1.000005f);
    //ReadTicker.attach(printfQueue.event(&Print_Sensors), 1.000005f);

    PPS.rise(GPSQueue.event(&GPS_data));
    PPS.fall(printfQueue.event(&Print_Sensors));
    
    wait(osWaitForever);
}
