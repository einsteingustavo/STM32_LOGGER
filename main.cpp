/*
    Data Logger implementation in STM32F103C8T6.
    Inputs:
        x3 Analog Inputs;
        x2 Digital (Frequency) Inputs;
    In this set, it is designed for a 200Hz sample rate.
    All the data are saved periodically (every 0.25s) to a folder in the SD card.
    To read the data, use the file "read_struct2.0.c" in the folder results.
    
   Implemented by Einstein "Hashtag" Gustavo(Electronics Coordinator 2019) at Mangue Baja Team, UFPE.
*/

#include "mbed.h"
#include <stdio.h>
#include <errno.h>
#include "SDBlockDevice.h"
#include "FATFileSystem.h"

#define BUFFER_SIZE 200                         // Acquisition buffer
#define SAVE_WHEN 50                            // Number of packets to save (fail safe)
#define SAMPLE_FREQ 200                         // Frequency in Hz

/* Debug */
PwmOut signal_wave(PB_3);                           // Debug wave to test frequency channels

/* I/O */
Serial pc(PA_2, PA_3);                              // Debug purposes
SDBlockDevice   sd(PB_15, PB_14, PB_13, PB_12);  // mosi, miso, sck, cs
FATFileSystem   fileSystem("sd");
DigitalOut warning(PA_12);                          // When device is ready, led is permanently OFF
DigitalOut logging(PA_15);                          // When data is beign acquired, led is ON
InterruptIn start(PB_4,PullUp);                            // Press button to start/stop acquisition
InterruptIn freq_chan1(PB_8,PullUp);                       // Frequency channel 1
InterruptIn freq_chan2(PB_9,PullUp);                       // Frequency channel 2
AnalogIn pot0(PB_1),
         pot1(PB_0),
         pot2(PA_7);

/* Data structure */
typedef struct
{
    uint16_t analog0;
    uint16_t analog1;
    uint16_t analog2;
    uint16_t pulses_chan1;
    uint16_t pulses_chan2;
    uint32_t time_stamp;
} packet_t;


Timer t;                                        // Device timer
Ticker acq;                                     // Acquisition timer interrupt source
CircularBuffer<packet_t, BUFFER_SIZE> buffer;   // Acquisition buffer
int buffer_counter = 0;                         // Packet currently in buffer
int err;                                        // SD library utility
bool running = false;                           // Device status
bool isAnalogReady = false;
uint16_t pulse_counter1 = 0,
         pulse_counter2 = 0,                    // Frequency counter variables
         last_acq;                              // Time of last acquisition
packet_t acq_pck;                               // Current data packet

void sampleISR();                               // Data acquisition ISR
uint32_t count_files_in_sd(const char *fsrc);   // Compute number of files in SD
void freq_channel1_ISR();                       // Frequency counter ISR, channel 1
void freq_channel2_ISR();                       // Frequency counter ISR, channel 2
void toggle_logging();                          // Start button ISR

int main()
{   
    pc.printf("\r\niniciou\r\n");
    logging = 0;                                // logging led OFF
    int num_parts = 0,                          // Number of parts already saved
        num_files = 0,                          // Number of files in SD
        svd_pck = 0;                            // Number of saved packets (in current part)
    char name_dir[12];                          // Name of current folder (new RUN)
    char name_file[20];                         // Name of current file (partX)
    FILE* fp;                                   
    packet_t temp;
    signal_wave.period_us(50);
    signal_wave.write(0.5f);
    
    /* Wait for SD mount */
    do
    {
        /* Try to mount the filesystem */
        pc.printf("Mounting the filesystem... ");
        fflush(stdout);

        err = fileSystem.mount(&sd);
        pc.printf("%s\n", (err ? "Fail :(" : "OK"));
        if (err)
        {
            /* Reformat if we can't mount the filesystem
            this should only happen on the first boot */
            pc.printf("No filesystem found, formatting... ");
            fflush(stdout);
            err = fileSystem.reformat(&sd);
            pc.printf("%s\n", (err ? "Fail :(" : "OK"));
            if (err) 
            {
                error("error: %s (%d)\n", strerror(-err), err);
            }
        }
    }while(err);
    
    pc.printf("\r\nDebug 1\r\n");
    
    pc.printf("\r\nDebug 2\r\n");
    
    num_files = count_files_in_sd("/sd");
    sprintf(name_dir, "%s%d", "/sd/RUN", num_files + 1);
    
    pc.printf("\r\nDebug 3\r\n");
    pc.printf("\r\nnum_files = %d\r\n", num_files);
    
    start.fall(&toggle_logging);                    // Attach start button ISR
    
    while(!running)                                 // Wait button press
    {                                
        warning = 1;                                
        pc.printf("\r\nrunning=%d\r\n", running);   // For some reason if this line is empty the code doesn't run
    }
    start.fall(NULL);
    
    /* Create RUN directory */
    mkdir(name_dir, 0777);
    warning = 0;                                // Warning led OFF
    //sprintf(name_file, "%s%s%d", name_dir, "/part", num_parts++);
    sprintf(name_file, "%s%s%d", name_dir, "/part", num_parts+1);
    fp = fopen(name_file, "a");                 // Creates first data file
    t.start();                                  // Start device timer
    freq_chan1.fall(&freq_channel1_ISR);
    freq_chan2.fall(&freq_channel2_ISR);
    acq.attach(&sampleISR, 1.0/SAMPLE_FREQ);    // Start data acquisition
    logging = 1;                                // logging led ON
        
    while(running)
    {

        if(buffer.full())
        {
            fclose(fp);
            warning = 1;                        // Turn warning led ON if buffer gets full (abnormal situation)
            pc.putc('X');                       // Debug message
        }
        else if(!buffer.empty())
        {   
            
            if(isAnalogReady)
            {
                acq_pck.analog0 = pot0.read_u16();          // Read analog sensor 0            
                acq_pck.analog1 = pot1.read_u16();          // Read analog sensor 1
                acq_pck.analog2 = pot2.read_u16();          // Read analog sensor 2
                isAnalogReady = false;
            }
            
            pc.putc('G');                       // Debug message
            
            /* Remove packet from buffer and writes it to file */
            buffer.pop(temp);                
            buffer_counter--;
            fwrite((void *)&temp, sizeof(packet_t), 1, fp);
            svd_pck++;
            
            /* Create new data file 
            (Deactivated because the code doesn't works fine doing that so many times) */
            if(svd_pck == SAVE_WHEN)
            {   
                //fclose(fp);
                //sprintf(name_file, "%s%s%d", name_dir, "/part", num_parts++);
                //t2 = t.read_ms();
                //fp = fopen(name_file, "w");
                //printf("t2=%d\r\n",(t.read_ms()-t2));
                //pc.printf("%d\r\n", buffer_counter);  // Debug message
                svd_pck = 0;
            }
        }
        
        /* Software debounce for start button */
        if((t.read_ms() > 10) && (t.read_ms() < 500))
            start.fall(toggle_logging);
    }
    
    /* Reset device if start button is pressed while logging */
    fclose(fp);
    logging = 0;
    NVIC_SystemReset();
    return 0;
}

void sampleISR()
{
    isAnalogReady = true;
    last_acq = t.read_ms();                     
    
    acq_pck.pulses_chan1 = pulse_counter1;      // Store frequence channel 1
    acq_pck.pulses_chan2 = pulse_counter2;      // Store frequence channel 2
    acq_pck.time_stamp = t.read_ms();           // Timestamp of data acquistion
    
    pulse_counter1= 0;
    pulse_counter2= 0;
    buffer.push(acq_pck);
    buffer_counter++;
}

uint32_t count_files_in_sd(const char *fsrc)
{   
    DIR *d = opendir(fsrc);
    struct dirent *p;
    uint32_t counter = 0;
    
    while ((p = readdir(d)) != NULL)   
    {
        if(strcmp(p->d_name, ".Trash-1000"))
            counter++;
    }
    closedir(d);
    return counter;
}

void freq_channel1_ISR()
{
    pulse_counter1++;
}

void freq_channel2_ISR()
{
    pulse_counter2++;
}

void toggle_logging()
{
    running = !running;
}
