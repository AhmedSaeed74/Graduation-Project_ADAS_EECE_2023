/*===============================================RTOS Configurations ====================================*/


//Define task priorities 
#define UART_RECIEVE_PRIORITY 			(configMAX_PRIORITIES - 1)
#define UART_SEND_PRIORITY        		(configMAX_PRIORITIES - 2)
#define MAKE_ALGORITHM_PRIORITY         (configMAX_PRIORITIES - 3)
#define ABS_UPDATE_PRIORITY             (configMAX_PRIORITIES - 4)

// Define task stack sizes 
#define UART_RECIEVE_STACK_SIZE 		(configMINIMAL_STACK_SIZE * 2)
#define UART_SEND_STACK_SIZE        	(configMINIMAL_STACK_SIZE * 2)
#define MAKE_ALGORITHM_STACK_SIZE       (configMINIMAL_STACK_SIZE * 2)
#define ABS_UPDATE_STACK_SIZE           (configMINIMAL_STACK_SIZE * 2)



/*
typedef enum  {
  osPriorityIdle          = -3,          ///< priority: idle (lowest)
  osPriorityLow           = -2,          ///< priority: low
  osPriorityBelowNormal   = -1,          ///< priority: below normal
  osPriorityNormal        =  0,          ///< priority: normal (default)
  osPriorityAboveNormal   = +1,          ///< priority: above normal
  osPriorityHigh          = +2,          ///< priority: high
  osPriorityRealtime      = +3,          ///< priority: realtime (highest)
  osPriorityError         =  0x84        ///< system cannot determine priority or thread has illegal priority
} osPriority;
*/
/*

//Define task priorities 
#define UART_RECIEVE_PRIORITY 			osPriorityNormal
#define UART_SEND_PRIORITY        		osPriorityNormal
#define MAKE_ALGORITHM_PRIORITY         osPriorityNormal
#define ABS_UPDATE_PRIORITY             osPriorityNormal

// Define task stack sizes 
#define UART_RECIEVE_STACK_SIZE 		(128 * 2)
#define UART_SEND_STACK_SIZE        	(128 * 2)
#define MAKE_ALGORITHM_STACK_SIZE       (128 * 2)
#define ABS_UPDATE_STACK_SIZE           (128 * 2)
*/

/*=============================================== APP Configurations ====================================*/


/* Enumeration for ABS states */
typedef enum {
    ABS_STATE_NORMAL,
    ABS_STATE_WARNING,
    ABS_STATE_DECREASING_SPEED,
    ABS_STATE_FULL_BRAKING
} ABS_State_t;

typedef struct {
	char Warning_Flag;
	char Decreasing_Speed_Flag;
	char Full_Braking_Flag;
	char RangeThresholdFlag;
	char Turn_Right;
	char Turn_Left;
	char LaneCenterRight;
	char LaneCenterLeft;
}flags_t;
/* Define the message structure for passing sensor data between tasks */
typedef struct {
    float desiredSpeed;
    float Heading;
    float rangeOfLRR;
    float relativeSpeedOfLRR;
    float rangeOfSRR;
    float relativeSpeedOfSRR;
    float rangeOfFRR;
    float relativeSpeedOfFRR;
    float azimuthAngleOfFRR;
    float rangeOfBRR;
    float relativeSpeedOfBRR;
    float azimuthAngleOfBRR;
    float rangeOfFLR;
    float relativeSpeedOfFLR;
    float azimuthAngleOfFLR;
    float rangeOfBLR;
    float relativeSpeedOfBLR;
    float azimuthAngleOfBLR;
    float currentVelocity;
    float laneMarkerCoordX_D0;
    float laneMarkerCoordY_D0;
    float laneMarkerCoordX_D1;
    float laneMarkerCoordY_D1;
    float rightWorldCoordX_D0;
    float rightWorldCoordY_D0;
    float rightWorldCoordX_D1;
    float rightWorldCoordY_D1;
    float laneMarkerLeftIntersection_D1;
    float laneMarkerRightIntersection_D1;
    int roadIDunderLaneMarker;
} InputData_t;

/* Define the message structure for passing TTC values between tasks */
typedef struct
{
	float TTC_SRR;
	float TTC_LRR;
	float TTC_FRR;
	float TTC_BRR;
	float TTC_FLR;
	float TTC_BLR;
} TTCCalculation_t;

/* Define the message structure for passing control values between tasks */
typedef struct
{
	int control;
    int brake;
    int steeringAngle;
    int laneID;
    int laneKeeping;
    int laneMarkerAngle;
    int driver_drowsiness;
    int driver_distraction;
} ControlData_t;




/*Constants for Timing thresholds */
#define TIME_WARNING_THRESHOLD            10.0
#define TIME_40_PRESENT_BRAKING_THRESHOLD 10.0
#define TIME_FULL_BRAKE_THRESHOLD          5.0


#define MAX_FORCE 150

/*Task Prototypes*/


void vTaskReceiveUART(void * argument);

void vTaskMakeAlgorithm(void * argument);

void vTaskUpdateABS(void *argument);

void vTaskSendUART(void *argument);

void vTaskReceiveCAN(void *argument);

/* Function declarations for ABS states */
ABS_State_t ABS_Normal_State(float distance);
ABS_State_t ABS_Warning_State(float relative_velocity);
ABS_State_t ABS_Decreasing_Speed_State(float distance);
ABS_State_t ABS_Full_Braking_State(void);
