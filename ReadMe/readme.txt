/**
  ******************************************************************************
  * File Name          : readme.txt
  *	Author			   : Endeavor
  * Description        : Introduction for this program
  ******************************************************************************
  * Copyright (c) 2017 CSU_FSAE 
  * All rights reserved.
  *
  ******************************************************************************
  **/
  
	Ã¶¾ÙÀàĞÍ±àºÅ:
	Can.h						:1~149						(Î´Ê¹ÓÃµÄÎª±£Áô±àºÅ)
	Data_Generic.h				:150~199
	spi.h						:200~249
	HostPCDebug_AT24CxxMem.h	:250~299	
	GPS_UBLOX_NEO_M8N.h			:300~349
	
	Fram´æ´¢Æ÷£¬µØÖ··ÖÅä:
	(Õ¼Á½¸ö×Ö½Ú)
	BrakePressure_Range_High_ID					0x00 				//ÖÆ¶¯Ì¤°åÓÍÑ¹´«¸ĞÆ÷Ñ¹Á¦Á¿³Ì¸ßÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
	BrakePressure_Range_Low_ID					0x02				//ÖÆ¶¯Ì¤°åÓÍÑ¹´«¸ĞÆ÷Ñ¹Á¦Á¿³ÌµÍÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
	BrakePressure_Range_ActualHigh_ID			0x04				//ÖÆ¶¯Ì¤°åÓÍÑ¹´«¸ĞÆ÷Êµ¼Ê¸ßID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
	BrakePressure_Range_ActualLow_ID			0x06				//ÖÆ¶¯Ì¤°åÓÍÑ¹´«¸ĞÆ÷Êµ¼ÊµIDÍ£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
	AccPeda_Range_High_ID						0x08				//¼ÓËÙÌ¤°åĞĞ³ÌÉÏÏŞÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
	AccPeda_Range_Low_ID						0x0A				//¼ÓËÙÌ¤°åĞĞ³ÌÏÂÏŞÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
	AccPeda_InitialDiff_Of_LandR_ID				0x0C				//×óÓÒ¼ÓËÙÌ¤°å³õÊ¼Ê±²îÒìÖµID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
	SterringWheelAngle_Range_ID		 			0x0E				//·½ÏòÅÌ×ª½Ç´«¸ĞÆ÷×î´ó×ª¶¯½Ç¶ÈÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
	SterringWheel_ADMiddle_ID					0x10				//·½ÏòÅÌÖĞ¼äÎ»ÖÃADÊı×ÖÁ¿ID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
	SterringWheel_TurnRange_ID					0x12				//·½ÏòÅÌ×óÓÒ×ª¶¯·¶Î§ADÊı×ÖÁ¿ ID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
	MotCtrllerTempThreshold_High_ID				0x14				//¿ØÖÆË®±Ã¿ªÆôµÄµç»ú¿ØÖÆÆ÷ÎÂ¶ÈÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
	MotCtrllerTempThreshold_Low_ID				0x16				//¿ØÖÆË®±Ã¹Ø±ÕµÄµç»ú¿ØÖÆÆ÷ÎÂ¶ÈÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
	Line_Corners_TCSMode_Angle_ID				0x18				//¿ØÖÆÇ£ÒıÁ¦Ä£Ê½Ñ¡ÔñµÄ·½ÏòÅÌ½Ç¶ÈÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
	DRS_SteeringWheel_Angle_ID					0x1A				//¿ØÖÆDRS¿ªÆôµÄ·½ÏòÅÌ½Ç¶ÈÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
	Close_DRS_AdjustAngle_ID					0x1C				//¿ªÆôDRSÊ±£¬Î²Òí±£³ÖµÄ½Ç¶ÈÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
	Open_DRS_AdjustAngle_ID						0x1E				//¹Ø±ÕDRSÊ±£¬Î²Òí±£³ÖµÄ½Ç¶ÈÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
	TCS_PID_Kp_ID								0x20				//Ç£ÒıÁ¦¿ØÖÆ±ÈÀıÏµÊıÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
	TCS_PID_Ki_ID								0x22				//Ç£ÒıÁ¦¿ØÖÆ»ı·ÖÏµÊıÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
	TCS_PID_Kd_ID								0x24				//Ç£ÒıÁ¦¿ØÖÆÎ¢·ÖÏµÊıÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
	OptimalSlipRate_ID							0x26				//Ç£ÒıÁ¦¿ØÖÆ×îÓÅ»¬ÒÆÂÊÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
	SuspensionLineDistF1_ADMiddle_ID			0x28				//Ç°Ğü¼ÜÏßÎ»ÒÆ×óµÄÖĞ¼äÎ»ÖÃADÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
	SuspensionLineDistF2_ADMiddle_ID			0x2A				//Ç°Ğü¼ÜÏßÎ»ÒÆÓÒµÄÖĞ¼äÎ»ÖÃADÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
	SuspensionLineDistB1_ADMiddle_ID			0x2C				//ºóĞü¼ÜÏßÎ»ÒÆ×óµÄÖĞ¼äÎ»ÖÃADÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
	SuspensionLineDistB2_ADMiddle_ID			0x2E				//ºóĞü¼ÜÏßÎ»ÒÆÓÒµÄÖĞ¼äÎ»ÖÃADÊı¾İID£¬ÓÃÓÚÎŞÏß´®¿Ú½ÓÊÕ&·¢ËÍ
	
	Mileage_ID 									0xFF				//Àï³ÌÊı¾İ
	
	(Õ¼Ò»¸ö×Ö½Ú)
	LeakageCurrentLimit_State_Data_ID			0x0101				//IMD Â©µç³¬ÏŞ ×´Ì¬Êı¾İ					
	
	
	