#include "JHPWMPCA9685.h"
#include <math.h>
#include <string>
#include <cstring>

 PCA9685::PCA9685(int address) {
    kI2CBus = 0;           // Default I2C bus for Jetson TK1
    kI2CAddress = address ; // Defaults to 0x40 for PCA9685 ; jumper settable
    error = 0 ;
}

PCA9685::~PCA9685() {
    closePCA9685() ;
}

bool PCA9685::openPCA9685(std::string temp_name)
{
    const char *fileNameBuffer = temp_name.c_str();
    
    //sprintf(fileNameBuffer,"/dev/i2c-%d", kI2CBus);
    
    kI2CFileDescriptor = open(fileNameBuffer, O_RDWR);
    if (kI2CFileDescriptor < 0) {
        // Could not open the file
       error = errno ;
       return false ;
    }
    if (ioctl(kI2CFileDescriptor, I2C_SLAVE, kI2CAddress) < 0) {
        // Could not open the device on the bus
        error = errno ;
        return false ;
    }
    
    // Open Relay
    jetsonTX1GPIONumber relay = gpio219;
    gpioExport(relay);
    gpioSetDirection(relay,outputPin);
    if (!gpioSetValue(relay, on))
	std::cout << "GPIO" << relay << " has turn on" << std::endl;
    else 
	std::cout << "fail to open GPIO" << relay << std::endl;
    
    return true ;
}

void PCA9685::closePCA9685()
{
    if (kI2CFileDescriptor > 0) {
        close(kI2CFileDescriptor);
        // WARNING - This is not quite right, need to check for error first
        kI2CFileDescriptor = -1 ;
    }

    jetsonTX1GPIONumber relay = gpio219;
    gpioExport(relay);
    gpioSetDirection(relay,outputPin);
    if (!gpioSetValue(relay, off))
	std::cout << "GPIO" << relay << " has turn off" << std::endl;
    else 
	std::cout << "fail to close GPIO" << relay << std::endl;
    
}

void PCA9685::reset () {
    writeByte(PCA9685_MODE1, PCA9685_ALLCALL );
    writeByte(PCA9685_MODE2, PCA9685_OUTDRV) ;
    // Wait for oscillator to stabilize
    usleep(5000) ;
}

// Sets the frequency of the PWM signal
// Frequency is ranged between 40 and 1000 Hertz
void PCA9685::setPWMFrequency ( float frequency ) {
    printf("Setting PCA9685 PWM frequency to %f Hz\n",frequency) ;
    float rangedFrequency = fmin(fmax(frequency,40),1000) ;
    int prescale = (int)(25000000.0f / (4096 * rangedFrequency) - 0.5f) ;
    // For debugging
    // printf("PCA9685 Prescale: 0x%02X\n",prescale) ;
    int oldMode = readByte(PCA9685_MODE1) ;
     int newMode = ( oldMode & 0x7F ) | PCA9685_SLEEP ;
    writeByte(PCA9685_MODE1, newMode) ;
    writeByte(PCA9685_PRE_SCALE, prescale) ;
    writeByte(PCA9685_MODE1, oldMode) ;
    // Wait for oscillator to stabilize
    usleep(5000) ;
    writeByte(PCA9685_MODE1, oldMode | PCA9685_RESTART) ;
}

// Channels 0-15
// Channels are in sets of 4 bytes
void PCA9685::setPWM ( int channel, int onValue, int offValue) {
    writeByte(PCA9685_LED0_ON_L+4*channel, onValue & 0xFF) ;
    writeByte(PCA9685_LED0_ON_H+4*channel, onValue >> 8) ;
    writeByte(PCA9685_LED0_OFF_L+4*channel, offValue & 0xFF) ;
    writeByte(PCA9685_LED0_OFF_H+4*channel, offValue >> 8) ;
}

void PCA9685::setAllPWM (int onValue, int offValue) {
    writeByte(PCA9685_ALL_LED_ON_L, onValue & 0xFF) ;
    writeByte(PCA9685_ALL_LED_ON_H, onValue >> 8) ;
    writeByte(PCA9685_ALL_LED_OFF_L, offValue & 0xFF) ;
    writeByte(PCA9685_ALL_LED_OFF_H, offValue >> 8) ;
}


// Read the given register
int PCA9685::readByte(int readRegister)
{
    int toReturn = i2c_smbus_read_byte_data(kI2CFileDescriptor, readRegister);
    if (toReturn < 0) {
        printf("PCA9685 Read Byte error: %d",errno) ;
        error = errno ;
        toReturn = -1 ;
    }
    // For debugging
    // printf("Device 0x%02X returned 0x%02X from register 0x%02X\n", kI2CAddress, toReturn, readRegister);
    return toReturn ;
}

// Write the the given value to the given register
int PCA9685::writeByte(int writeRegister, int writeValue)
{   // For debugging:
    // printf("Wrote: 0x%02X to register 0x%02X \n",writeValue, writeRegister) ;
    int toReturn = i2c_smbus_write_byte_data(kI2CFileDescriptor, writeRegister, writeValue);
    if (toReturn < 0) {
        printf("PCA9685 Write Byte error: %d",errno) ;
	printf ("toReturn : %d" , toReturn);
        error = errno ;
        toReturn = -1 ;
    }
    return toReturn ;
}

//******************
//***GPIO library***
//******************

//
// gpioExport
// Export the given gpio to userspace;
// Return: Success = 0 ; otherwise open file error
int gpioExport ( jetsonGPIO gpio )
{
    int fileDescriptor, length;
    char commandBuffer[MAX_BUF];

    fileDescriptor = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
    if (fileDescriptor < 0) { 
        char errorBuffer[128] ;
        snprintf(errorBuffer,sizeof(errorBuffer), "gpioExport unable to open gpio%d",gpio) ;
        perror(errorBuffer);
        return fileDescriptor;
    }

    length = snprintf(commandBuffer, sizeof(commandBuffer), "%d", gpio);
    if (write(fileDescriptor, commandBuffer, length) != length) {
        perror("gpioExport"); 
        return fileDescriptor ;

    }
    close(fileDescriptor);

    return 0;
}

//
// gpioUnexport
// Unexport the given gpio from userspace
// Return: Success = 0 ; otherwise open file error
int gpioUnexport ( jetsonGPIO gpio )
{
    int fileDescriptor, length;
    char commandBuffer[MAX_BUF];

    fileDescriptor = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
    if (fileDescriptor < 0) { 
        char errorBuffer[128] ;
        snprintf(errorBuffer,sizeof(errorBuffer), "gpioUnexport unable to open gpio%d",gpio) ;
        perror(errorBuffer);
        return fileDescriptor;
    }

    length = snprintf(commandBuffer, sizeof(commandBuffer), "%d", gpio);
    if (write(fileDescriptor, commandBuffer, length) != length) {
        perror("gpioUnexport") ;
        return fileDescriptor ;
    }
    close(fileDescriptor);
    return 0;
}


// gpioSetDirection
// Set the direction of the GPIO pin 
// Return: Success = 0 ; otherwise open file error
int gpioSetDirection ( jetsonGPIO gpio, unsigned int out_flag )
{
    int fileDescriptor;
    char commandBuffer[MAX_BUF];

    snprintf(commandBuffer, sizeof(commandBuffer), SYSFS_GPIO_DIR  "/gpio%d/direction", gpio);

    fileDescriptor = open(commandBuffer, O_WRONLY);
    if (fileDescriptor < 0) { 
        char errorBuffer[128] ;
        snprintf(errorBuffer,sizeof(errorBuffer), "gpioSetDirection unable to open gpio%d",gpio) ;
        perror(errorBuffer);
        return fileDescriptor;
    }

    if (out_flag) {
        if (write(fileDescriptor, "out", 4) != 4) {
            perror("gpioSetDirection") ;
            return fileDescriptor ;
        }
    }
    else { 
        if (write(fileDescriptor, "in", 3) != 3) {
            perror("gpioSetDirection") ;
            return fileDescriptor ;
        }
    }
    close(fileDescriptor);
    return 0;
}


//
// gpioSetValue
// Set the value of the GPIO pin to 1 or 0
// Return: Success = 0 ; otherwise open file error
int gpioSetValue ( jetsonGPIO gpio, unsigned int value )
{
    int fileDescriptor;
    char commandBuffer[MAX_BUF];

    snprintf(commandBuffer, sizeof(commandBuffer), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

    fileDescriptor = open(commandBuffer, O_WRONLY);
    if (fileDescriptor < 0) {
        char errorBuffer[128] ;
        snprintf(errorBuffer,sizeof(errorBuffer), "gpioSetValue unable to open gpio%d",gpio) ;
        perror(errorBuffer);
        return fileDescriptor;
    }

    if (value) {
        if (write(fileDescriptor, "1", 2) != 2) {
            perror("gpioSetValue") ;
            return fileDescriptor ;
        }
    }
    else {
        if (write(fileDescriptor, "0", 2) != 2) {
            perror("gpioSetValue") ;
            return fileDescriptor ;
        }
    }
    close(fileDescriptor);
    return 0;
}

//
// gpioGetValue
// Get the value of the requested GPIO pin ; value return is 0 or 1
// Return: Success = 0 ; otherwise open file error
int gpioGetValue ( jetsonGPIO gpio, unsigned int *value)
{
    int fileDescriptor;
    char commandBuffer[MAX_BUF];
    char ch;

    snprintf(commandBuffer, sizeof(commandBuffer), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

    fileDescriptor = open(commandBuffer, O_RDONLY);
    if (fileDescriptor < 0) {
        char errorBuffer[128] ;
        snprintf(errorBuffer,sizeof(errorBuffer), "gpioGetValue unable to open gpio%d",gpio) ;
        perror(errorBuffer);
        return fileDescriptor;
    }

    if (read(fileDescriptor, &ch, 1) != 1) {
        perror("gpioGetValue") ;
        return fileDescriptor ;
     }

    if (ch != '0') {
        *value = 1;
    } else {
        *value = 0;
    }

    close(fileDescriptor);
    return 0;
}

//
// gpioSetEdge
// Set the edge of the GPIO pin
// Valid edges: 'none' 'rising' 'falling' 'both'
// Return: Success = 0 ; otherwise open file error
int gpioSetEdge ( jetsonGPIO gpio, char *edge )
{
    int fileDescriptor;
    char commandBuffer[MAX_BUF];

    snprintf(commandBuffer, sizeof(commandBuffer), SYSFS_GPIO_DIR "/gpio%d/edge", gpio);

    fileDescriptor = open(commandBuffer, O_WRONLY);
    if (fileDescriptor < 0) {
        char errorBuffer[128] ;
        snprintf(errorBuffer,sizeof(errorBuffer), "gpioSetEdge unable to open gpio%d",gpio) ;
        perror(errorBuffer);
        return fileDescriptor;
    }

    if (write(fileDescriptor, edge, strlen(edge) + 1) != ((int)(strlen(edge) + 1))) {
        perror("gpioSetEdge") ;
        return fileDescriptor ;
    }
    close(fileDescriptor);
    return 0;
}

//
// gpioOpen
// Open the given pin for reading
// Returns the file descriptor of the named pin
int gpioOpen( jetsonGPIO gpio )
{
    int fileDescriptor;
    char commandBuffer[MAX_BUF];

    snprintf(commandBuffer, sizeof(commandBuffer), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

    fileDescriptor = open(commandBuffer, O_RDONLY | O_NONBLOCK );
    if (fileDescriptor < 0) {
        char errorBuffer[128] ;
        snprintf(errorBuffer,sizeof(errorBuffer), "gpioOpen unable to open gpio%d",gpio) ;
        perror(errorBuffer);
    }
    return fileDescriptor;
}

//
// gpioClose
// Close the given file descriptor 
int gpioClose ( int fileDescriptor )
{
    return close(fileDescriptor);
}

// gpioActiveLow
// Set the active_low attribute of the GPIO pin to 1 or 0
// Return: Success = 0 ; otherwise open file error
int gpioActiveLow ( jetsonGPIO gpio, unsigned int value )
{
    int fileDescriptor;
    char commandBuffer[MAX_BUF];

    snprintf(commandBuffer, sizeof(commandBuffer), SYSFS_GPIO_DIR "/gpio%d/active_low", gpio);

    fileDescriptor = open(commandBuffer, O_WRONLY);
    if (fileDescriptor < 0) {
        char errorBuffer[128] ;
        snprintf(errorBuffer,sizeof(errorBuffer), "gpioActiveLow unable to open gpio%d",gpio) ;
        perror(errorBuffer);
        return fileDescriptor;
    }

    if (value) {
        if (write(fileDescriptor, "1", 2) != 2) {
            perror("gpioActiveLow") ;
            return fileDescriptor ;
        }
    }
    else {
        if (write(fileDescriptor, "0", 2) != 2) {
            perror("gpioActiveLow") ;
            return fileDescriptor ;
        }
    }
    close(fileDescriptor);
    return 0;
}












