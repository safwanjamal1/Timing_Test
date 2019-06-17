#include "mbed.h"
#include "MBed_Adafruit_GPS.h"
#include "XNucleoIKS01A2.h"

//Prints an error message location if fatal error occurs.
#define MBED_CONF_PLATFORM_ERROR_FILENAME_CAPTURE_ENABLED 0

//Configures pins and serial port.
Serial pc(USBTX, USBRX);
InterruptIn PPS(A1, PullNone);
DigitalOut led(LED1);

/* Sensor Board Variables */
uint8_t id;
float temp1, temp2, humid1, humid2;
char buffer1[32], buffer2[32], buffer3[32], buffer4[32];
int32_t axes1[3], axes2[3], axes3[3], axes4[3];

/* Instantiate the expansion board */
static XNucleoIKS01A2 *mems_expansion_board = XNucleoIKS01A2::instance(D14, D15, D4, D5);

/* Retrieve the composing elements of the expansion board */
static LSM303AGRMagSensor *magnetometer = mems_expansion_board->magnetometer;
static HTS221Sensor *hum_temp = mems_expansion_board->ht_sensor;
static LPS22HBSensor *press_temp = mems_expansion_board->pt_sensor;
static LSM6DSLSensor *acc_gyro = mems_expansion_board->acc_gyro;
static LSM303AGRAccSensor *accelerometer = mems_expansion_board->accelerometer;

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
int64_t usTime2 = 0;

int int_GPStime=0, int_count=0, sense_count=0;
time_t whattime;
bool UpdateTime = true;

/* Helper function for printing floats & doubles */
static char *print_double(char* str, double v, int decimalDigits=2)
{
    int i = 1;
    int intPart, fractPart;
    int len;
    char *ptr;

    /* prepare decimal digits multiplicator */
    for (;decimalDigits!=0; i*=10, decimalDigits--);

    /* calculate integer & fractinal parts */
    intPart = (int)v;
    fractPart = (int)((v-(double)(int)v)*i);

    /* fill in integer part */
    sprintf(str, "%i.", intPart);

    /* prepare fill in of fractional part */
    len = strlen(str);
    ptr = &str[len];

    /* fill in leading fractional zeros */
    for (i/=10;i>1; i/=10, ptr++) {
        if (fractPart >= i) {
        break;
        }
        *ptr = '0';
    }

    /* fill in (rest of) fractional part */
    sprintf(ptr, "%i", fractPart);

    return str;
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

void read_sensors(){
    hum_temp->get_temperature(&temp1);
    hum_temp->get_humidity(&humid1);
    press_temp->get_temperature(&temp2);
    press_temp->get_pressure(&humid2);
    magnetometer->get_m_axes(axes1);
    accelerometer->get_x_axes(axes2);
    acc_gyro->get_x_axes(axes3);
    acc_gyro->get_g_axes(axes4);
}
/* Prints to the serial console */
//This runs in the low priority thread
void Print_Sensors() {
    read_sensors();
    sense_count++;
    //Next two lines print current uc and gps time, possibly disadvantageous to do this rather than save them in variables
    //when us timer is saved/reset
    if (sense_count > 1)
    {
        pc.printf("%d ", time(NULL));
        pc.printf("%d ", int_GPStime);
        pc.printf("%lld", usTime1);
        pc.printf("%7s %s ", print_double(buffer1, temp1), print_double(buffer2, humid1));
        pc.printf("%7s %s ", print_double(buffer3, temp2), print_double(buffer4, humid2));
        pc.printf("%6ld %6ld %6ld ", axes1[0], axes1[1], axes1[2]);
        pc.printf("%6ld %6ld %6ld", axes2[0], axes2[1], axes2[2]);
        pc.printf("%6ld %6ld %6ld", axes3[0], axes3[1], axes3[2]);
        pc.printf("%6ld %6ld %6ld\r\n", axes4[0], axes4[1], axes4[2]);
    }
    else
    {
        UpdateTime = true;
    }
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

    /* Enable all sensors */
    hum_temp->enable();
    press_temp->enable();
    magnetometer->enable();
    accelerometer->enable();
    acc_gyro->enable_x();
    acc_gyro->enable_g();
    wait(1);

    /* Query CPU Clock */
    pc.printf("CPU SystemCoreClock is %d Hz\r\n", SystemCoreClock);
    pc.printf("Get Sysclk Source: %d\r\n", __HAL_RCC_GET_SYSCLK_SOURCE());
    pc.printf("HSI Clock Status: %d\r\n", RCC_OSCILLATORTYPE_HSI);
    pc.printf("HSE Clock Status: %d\r\n", RCC_OSCILLATORTYPE_HSE);
    pc.printf("LSE Clock Status: %d\r\n", RCC_OSCILLATORTYPE_LSE);
    pc.printf("LSI Clock Status: %d\r\n", RCC_OSCILLATORTYPE_LSI);
    pc.printf("PLL Clock Status: %d\r\n", RCC_SYSCLKSOURCE_STATUS_PLLCLK);
    pc.printf("PLL Clock Source: %d\r\n", __HAL_RCC_GET_PLL_OSCSOURCE());

    /* resets and starts the timer */
    t.reset();
    t.start();
    pc.printf("Timer Reset and Started...\r\n");

    /* Wait for GPS to Sync and uc to start up and properly sync GPS time. */
    do{
        UpdateTime = true;
        GPS_data();   //queries the GPS
        pc.printf("Waiting for GPS Fix...%d\r\n",myGPS.fix);
        pc.printf("GPS Time...%d\r\n",int_GPStime);
        pc.printf("Micro Time...%d\r\n",time(NULL));
        pc.printf("Counter Number...%d\r\n\r\n", int_count);
        int_count++;

    }while(myGPS.fix == false);
    UpdateTime = false;

    //Prints headers for each measurement.
    pc.printf("\r\nuC GPS uS TEP1 HUM TEP2 PRES MAGX MAGY MAGZ AC1X AC1Y AC1Z AC2X AC2Y AC2Z GYRX GYRY GYRZ\r\n");

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
