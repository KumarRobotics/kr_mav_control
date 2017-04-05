#ifndef SDK_TELEMETRY
#define SDK_TELEMETRY

void SDK_jetiAscTecExampleUpdateDisplay(unsigned char state);
void SDK_jetiAscTecExampleKeyChange(unsigned char key);
void SDK_jetiAscTecExampleInit(void);
void SDK_jetiAscTecExampleRun(void);

//Emergency modes (these are the emergency procedures which will automatically be activated if the data link between R/C and UAV is lost).
//Go to the AscTec Wiki for further information about the procedures!

#define EM_SAVE								0x01	//"direct landing"
#define EM_SAVE_EXTENDED_WAITING_TIME		0x02	//"wait and land"
#define EM_RETURN_AT_PREDEFINED_HEIGHT		0x04	//"come home"
#define EM_RETURN_AT_MISSION_SUMMIT			0x08	//"come home high"


#endif /*SDK_TELEMETRY*/
