#include <math.h>
void setup() {
  // put your setup code here, to run once:

}

float L0_dist(int adc)
{
  adc = (float)adc;
  if(adc <= 1024 && adc > 987)
  {
    return ((-adc/74)+(512/37));  
  }
  else if(adc <=987 && adc > 932)
  {
    return ((-adc/110)+(521/55));  
  }
  else if(adc <= 932 && adc > 860)
  {
    return ((-adc/144)+(269/30));  
  }
  else if(adc <=860 && adc > 743 )
  {
    return ((-adc/117)+(2071/234));  
  }
  else if (adc <= 743 && adc > 521)
  {
    return (-adc/444)+(1853/444);  
  }
  else if((adc <=521 && adc >490) || (adc<=289 && adc >=200))
  {
    return (950/(adc-173.436));  
  }
  else if(adc <=490 && adc >289)
  {
    return (950/(adc-173.436))+.5;  
  }
  else return 36; // adc <200
}

float R0_dist(int adc)
{
  adc = (float)adc;
  if(adc > 907)
  {
    return 0;  
  }
  else if(adc <= 907 && adc >= 292)
  {
    return ((-1.0551*pow(10,16)*pow(adc,2) + 9.45793*pow(10,18)*adc + 1.03392*pow(10,20)) / (pow(adc,3) - 3.90697*pow(10,15)*pow(adc,2) + 4.62587*pow(10,18)*adc - 9.62419*pow(10,20)));  
  }
  else return 36; //adc < 292
}

float L90_dist(int adc)
{
  adc = (float)adc;
  if(adc > 900)
  {
    return 0;  
  }
  else if(adc <= 900 && adc >= 390)
  {
    return ((373.055*pow(adc,2)-645201*adc+2.80822*pow(10,8))/(pow(adc,3)-1922.85*pow(adc,2)+1.1734*pow(10,6)*adc-2.16857*pow(10,8)));  
  }
  else return 15; // adc < 390
}

float R90_dist(int adc)
{
  adc = (float)adc;
  if(adc > 876)
  {
    return 0;  
  }
  else if(adc <= 876 && adc > 829)
  {
    return ((-adc/94)+(438/47));  
  }
  else if (adc <= 829 && adc > 792)
  {
    return  ((-adc/74)+(433/37));
  }
  else if (adc <= 792 && adc > 757)
  {
    return  ((-adc/70)+(431/35));
  }
  else if (adc <= 757 && adc > 342)
  {
    return  ((369.787*adc-46951.9)/(pow(adc,2)-590.184*adc+90210.6));
  }
  else return 15; //adc < 342
  
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
