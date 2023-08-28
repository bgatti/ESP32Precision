#include "MissionControl.h"

QueueHandle_t MissionControltoGuidance;
static MissionState currentState = SLEEP;

void MissionControl_Init() {
    // Initialize MissionControltoGuidance queue
    MissionControltoGuidance = xQueueCreate(5, sizeof(MissionState));
}

void MissionControl_StateHandler(void *pvParameters) {
    // Initialize Mission Control
    MissionControl_Init();
    
    MissionState nextState = SLEEP;
    //Status status;
    
    while(1) {
        switch(currentState) {
            case SLEEP:
                // Handle Sleep actions
                break;
            
            case TEST:
                // Handle Test actions
                break;
                
            case ERROR:
                // Handle Error actions
                break;
                
            case STANDBY:
                // Handle Standby actions
                break;

            case STABLE:
                // Handle Stable actions
                break;

            case ACTIVE:
                // Handle Active actions
                break;

            default:
                break;
        }

        // Receive status message from Guidance
//        xQueueReceive(GuidancetoMissionControl, &status, portMAX_DELAY);
        
        // Update state based on status message
        // Your state transition logic goes here
        
        currentState = nextState;
    }
}
