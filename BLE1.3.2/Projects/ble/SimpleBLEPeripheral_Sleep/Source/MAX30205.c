#include "MAX30205.h" 
 
float tmpCaliK;
float tmpCaliB;
unsigned char tempDevice ; //0 : MAX    1: NTC

float GetTemperature(void)
{
  float temperature= 0.0f;
  int16_t raw = 0;
  uint8_t readRaw[2];
  HalI2CReadReg(MAX30205_ADDRESS,MAX30205_TEMPERATURE,2,readRaw);
  //printf("readRaw[0];:%d\r\n",readRaw[0]);
  //printf("readRaw[1];:%d\r\n",readRaw[1]);
  raw = ((uint16_t) readRaw[0] << 8) | readRaw[1];  //combine two bytes
  temperature = raw  * 0.00390625f;     // convert to temperature
  //printf("raw:%d\r\n",raw);
  return  temperature;
}
	
void MAX_shutdown(void)
{
  uint8_t reg;
  HalI2CReadReg(MAX30205_ADDRESS, MAX30205_CONFIGURATION,1,&reg);  // Get the current register
  reg |= 0x80;
  HalI2CWriteReg(MAX30205_ADDRESS, MAX30205_CONFIGURATION, 1, &reg);
}
void MAX_begin(void)
{
  HalI2CWriteReg(MAX30205_ADDRESS, MAX30205_CONFIGURATION, 1, 0x00); //mode config
  HalI2CWriteReg(MAX30205_ADDRESS, MAX30205_THYST, 1, 0x00); // set threshold
  HalI2CWriteReg(MAX30205_ADDRESS, MAX30205_TOS, 1, 0x00); //
}



const float __code wendu[106][2]={//
0,3274,//0 32.74k
1,3111,//1 31.11k
2,2957,//2 29.57k
3,2812,//
4,2674,//
5,2545,
6,2422,
7,2306,
8,2196,
9,2092,
10,1993,
11,1900,
12,1811,
13,1728,
14,1648,
15,1573,
16,1501,
17,1433,
18,1369,
19,1308,
20,1250,
21,1194,
22,1142,
23,1092,
24,1045,
25,1000,//25 10k                
26,957,//26 9.57k
27,916,
28,877,
29,840,
30,805,
31,771,
32,739,
33,709,
34,679,
35,652,
36,625,
37,600,
38,576,
39,553,
40,531,
41,510,
42,490,
43,471,
44,453,
45,435,
46,418,
47,402,
48,387,
49,372,
50,358,
51,345,
52,332,
53,320,
54,308,
55,297,
56,286,
57,276,
58,266,
59,256,
60,247,
61,238, 
62,230,
63,222,
64,214,
65,207,
66,199,
67,193,
68,186,
69,180,
70,174,
71,168,
72,162,
73,157,
74,152,
75,147,
76,142,
77,137,
78,133,
79,128,
80,124,
81,120,
82,116,
83,113,
84,109,
85,106,
86,102,//86 1.02k
87,99,//87 0.99k
88,96,
89,93,
90,90,
91,88,
92,85,
93,82,
94,80,
95,78,
96,75,
97,73,
98,71,
99,69,
100,67,
101,65,
102,63,
103,61, 
104,59,
105,58//105 0.58k
          

}; 

float GetNTCTemperature(void)
{
  float tmpf2,r_value;
  uint32_t AD_value,ii;
  //uint16 temp16;
  
  //tmpf1 = GetTemperature1();
//  HalAdcInit();
//  HalAdcSetReference( HAL_ADC_REF_AVDD );
  AD_value = HalAdcRead( HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_12 );
  AD_value  = 2800000/2048*AD_value/1000;
  r_value = (AD_value*1000)/(3300-AD_value);
 // temp16 = r_value;
 // HalLcdWriteStringValue( " r_value:", temp16, 10, HAL_LCD_LINE_8 );
 // tmpf2 = 0;
  for(ii=0;ii<106;ii++)  
  {
    if((r_value <= wendu[ii][1]) && (r_value > wendu[ii+1][1]))
    { //
            //dat=(wendu[ii][0]);//dat2
      tmpf2 = (float)((wendu[ii+1][0]-wendu[ii][0])/(wendu[ii][1]-wendu[ii+1][1]));
      tmpf2 = tmpf2*(wendu[ii][1]-r_value);
      tmpf2 = tmpf2 + wendu[ii][0];
      ii=0;
      break;
    }//else{dat=0;}
  }
  return tmpf2;
}

float GetTemperature1(void)
{
  float temperature= 0.0;
  float temperatureNTC= 0.0;
  float temperatureMAX= 0.0;

  temperatureMAX = GetTemperature();

  temperatureNTC = GetNTCTemperature() + 1.65f;
        
  if(temperatureMAX < 0.3f)
  {
    temperatureMAX = temperatureNTC;
  }
        
        
  if(temperatureNTC > temperatureMAX)
  {
          
    if((temperatureNTC - temperatureMAX) > 1.10f)
    {
      temperature = (temperatureMAX + temperatureNTC)/2;
      tempDevice = 0;
    }
    else if((temperatureNTC - temperatureMAX) > 0.50f)
    {
      temperature = temperatureNTC;
      tempDevice = 1;
    }
    else
    {
      temperature = temperatureMAX;
      tempDevice = 0;
    }

  }
  else if(temperatureNTC < temperatureMAX)
  {
    if((temperatureMAX - temperatureNTC) > 0.50f)
    {
      temperature = temperatureNTC;
      tempDevice = 1;
    }
    else if(((temperatureMAX - temperatureNTC) > 0.45f)&&((temperatureMAX - temperatureNTC) < 0.50f))
    {
        temperature = temperatureNTC + 0.25f;
        tempDevice = 1;
    }
    else if(((temperatureMAX - temperatureNTC) > 0.25f)&&((temperatureMAX - temperatureNTC) < 0.45f))
    {
        temperature = temperatureMAX - 0.25f;
        tempDevice = 0;
    }
    else 
    {
        temperature = temperatureMAX;
        tempDevice = 0;
    }
  }
  else 
  {
    temperature = temperatureNTC;
    tempDevice = 0;
  }
  //temperatureMAX = 0;
  if((temperatureMAX < 0.5f)&&(temperatureNTC > 0.5f))
  {
     temperature = temperatureNTC;
  }
  else if((temperatureNTC < 0.5f)&&(temperatureMAX > 0.5f))
  {
    temperature = temperatureMAX;
  }
  else if((temperatureNTC < 0.5f)&&(temperatureMAX < 0.5f))
  {
  }
    temperature = temperatureNTC;
  
  if(temperature > 37.0f)
  {
    temperature = temperature;
  }
  else
  {
    temperature = (float)(temperature * tmpCaliK);
    if(temperature < 0.1f)
    {
      temperature = (float)(temperature * tmpCaliK);
    }          
    temperature = (float)(temperature + tmpCaliB);
  }
    if((temperature > 50) || (temperature < 0))
      temperature = 44.4;
  return  temperature;
}








