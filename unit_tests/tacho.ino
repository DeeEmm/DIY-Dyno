
/***********************************************************
 * @name The DIY Dyno project
 * @details Dyno control and data acquisition based on ESP32 microprocessor and strain guage style load cell
 * @link https://diydyno.com
 * @author 
 * 
 * @file tacho.ino
 * 
 * @brief tacho unit tests
 * 
 * @remarks For more information please visit the WIKI on our GitHub project page: https://github.com/DeeEmm/DIY-Dyno/wiki
 * Or join our support forums: https://github.com/DeeEmm/DIY-Dyno/discussions
 * You can also visit our Facebook community: https://www.facebook.com/groups/diydyno/
 * 
 * Except where noted this project and all associated files are provided for use under the GNU GPL3 license:
 * https://github.com/DeeEmm/DIY-Dyno/blob/master/LICENSE
 * 
 *
 * DEPENDENCIES
 * This program has a number of core libraries that must be available for it to work. Please refer to platformio.ini
 * 
 *
 **/

#include <Arduino.h>


/***********************************************************
 * @brief Default setup function
 * @details Initialises system and sets up core tasks
 * @note We can assign tasks to specific cores if required (currently disabled)
 ***/
void setup(void) {

}


/***********************************************************
 * @brief MAIN LOOP
 * @details processes API requests 
 * @details pushes data to clients via SSE
 * 
 * @note Use non-breaking delays for throttling events
 ***/
void loop () {

}