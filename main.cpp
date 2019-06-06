#include "mbed.h"
#include "MBed_Adafruit_GPS.h"



#define MBED_CONF_PLATFORM_ERROR_FILENAME_CAPTURE_ENABLED 0

//Configures pins and serial port
Serial pc(USBTX, USBRX);
InterruptIn PPS(A1, PullNone);
DigitalOut led(LED1);

//Configure GPS
Serial * gps_Serial = new Serial(D1,D0); //serial object for use w/ GPS
Adafruit_GPS myGPS(gps_Serial); //object of Adafruit's GPS class
char c; //when read via Adafruit_GPS::read(), the class returns single character stored here

/* Defines the two queues used, one for events and one for printing to the screen */
EventQueue printfQueue;
EventQueue eventQueue;

/* Defines the timer */
Timer t;
time_t whattime;

int int_time=0, int_count=0;
int64_t usTime1 = 0, usTime2 = 0, usDeltaTime = 0;
bool UpdateTime = true;

void Detect1PPS() {
    led = !led;
}

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
void Print_Sensors() {
    // this runs in the lower priority thread
    pc.printf("%d ", whattime);
    pc.printf("%d ", int_time);
    pc.printf("%lld \r\n", usTime1);
}

/* Reads the sensor data */
void Read_Sensors() {
    Print_Sensors();
}

//Collects and parses GPS data
//This runs in the high priority thread
void GPS_data() {
    usTime1 = t.read_us();
    t.reset();
    printf("%d\r\n",myGPS.seconds);
    int_time = asUnixTime(myGPS.year+2000, myGPS.month, myGPS.day, myGPS.hour, myGPS.minute, myGPS.seconds);
    whattime=time(NULL);
    
    do{
        c = myGPS.read();   //queries the GPS
        //if (c) { printf("%c", c); }
        //check if we recieved a new message from GPS, if so, attempt to parse it,
        if ( myGPS.newNMEAreceived() ) {
            printf("new nmea received\r\n");
            if ( myGPS.parse(myGPS.lastNMEA()) ){
                    int_time = asUnixTime(myGPS.year+2000, myGPS.month, myGPS.day, myGPS.hour, myGPS.minute, myGPS.seconds);

                if (UpdateTime) {
                    set_time(int_time);
                    UpdateTime = false;
                }  
                break;
            }
        }
    }while( !myGPS.newNMEAreceived() );
    
    /*
    whattime = time(NULL);
    usTime2 = t.read_high_resolution_us();
    usDeltaTime = usTime2 - usTime1;
    t.stop();
    t.reset();
    t.start();
    usTime1 = t.read_high_resolution_us();
    UpdateTime = false;
    */
}

// main() runs in its own thread in the OS
int main()
{
    myGPS.begin(9600);
    
    //myGPS.sendCommand(PMTK_STANDBY);
    pc.printf("Entering GPS Standby...\r\n");
    wait(1);
    myGPS.sendCommand(PMTK_AWAKE);
    pc.printf("Wake Up GPS...\r\n");
    wait(1);
    myGPS.sendCommand(PMTK_SET_BAUD_57600);
    pc.printf("Set GPS Baud Rate to 57600...\r\n");
    myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    pc.printf("Set RM Message Format Only...\r\n");
    myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
    pc.printf("Set 10Hz Message Update Rate...\r\n");
    myGPS.sendCommand(PGCMD_NOANTENNA);
    pc.printf("Turn Off Antenna Messages...\r\n");

    /* resets and starts the timer */
    t.reset();
    t.start();
    pc.printf("Timer Reset and Started...\r\n");
    set_time(1);
    /*
    do{
        UpdateTime = true;
        GPS_data();   //queries the GPS
        pc.printf("Waiting for GPS Fix...%d\r\n",myGPS.fix);
        pc.printf("GPS Time...%d\r\n",int_time);
        pc.printf("Micro Time...%d\r\n",whattime);
        pc.printf("Counter Number...%d\r\n\r\n", int_count);
        int_count++;
        wait(1);
    }while((myGPS.fix == false) || (int_count < 4));
    UpdateTime = false;
    */

    //Prints headers for each measurement.
    pc.printf("\r\nuC GPS uS\r\n");
    
    // normal priority thread for other events
    Thread eventThread(osPriorityHigh);
    eventThread.start(callback(&eventQueue, &EventQueue::dispatch_forever));
  
    // low priority thread for calling printf()
    Thread printfThread(osPriorityLow);
    printfThread.start(callback(&printfQueue, &EventQueue::dispatch_forever));


    // call read_sensors 1 every second, automatically defering to the eventThread
    //Ticker GPSTicker;
    //Ticker ReadTicker;

    //GPSTicker.attach(eventQueue.event(&GPS_data), 1.000005f);
    //ReadTicker.attach(printfQueue.event(&Print_Sensors), 1.000005f);

    PPS.rise(eventQueue.event(&GPS_data));
    PPS.fall(printfQueue.event(&Print_Sensors));
    
    wait(osWaitForever);
}
