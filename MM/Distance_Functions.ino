#include <math.h>

float L0_dist(float adc)
{
  //adc = (float)adc;
  if(adc <= 1024 && adc > 987)
  {
    return ((-adc/74.00)+(512.00/37.00));  
  }
  else if(adc <=987 && adc > 932)
  {
    return ((-adc/110.00)+(521.00/55.00));  
  }
  else if(adc <= 932 && adc > 860)
  {
    return ((-adc/144.00)+(269.00/36.00));  
  }
  else if(adc <=860 && adc > 743 )
  {
    return ((-adc/117.00)+(2071.00/234.00));  
  }
  else if (adc <= 743 && adc > 521)
  {
    return (-adc/444.00)+(1853.00/444.00);  
  }
  else if(adc <=521 && adc >202)
  {
    return ((-5.39773*pow(10,13)*pow(adc,2) + 1.24476*pow(10,17)*adc - 2.1295*pow(10,19)) / (pow(adc,3) + 7.79374*pow(10,13)*pow(adc,2) - 2.5872*pow(10,16)*adc + 2.09414*pow(10,18)));  
  }
  else return 36.00; // adc < 202
}

float R0_dist(float adc)
{
  //adc = (float)adc;
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

float L90_dist(float adc)
{
  //adc = (float)adc;
  if(adc > 900)
  {
    return 0;  
  }
  else if(adc <= 900 && adc >= 379)
  {
    return ((373.055*pow(adc,2)-645201*adc+2.80822*pow(10,8))/(pow(adc,3)-1922.85*pow(adc,2)+1.1734*pow(10,6)*adc-2.16857*pow(10,8)));  
  }
  else return 15; // adc < 379
}

float R90_dist(float adc)
{
  if(adc > 900)
  {
    return 0;  
  }
  else if(adc <= 900 && adc > 843)
  {
    return ((150.00/19.00)-(adc/114.00))+.5;  
  }
  else if (adc <= 843 && adc > 807)
  {
    return  ((-(adc/72.00))+(293.00/24.00))+.2;
  }
  else if (adc <= 807 && adc > 776)
  {
    return  ((-adc/62.00)+(869.00/62.00))+.4;
  }
  else if (adc <= 776 && adc > 741)
  {
    return  ((-adc/70.00)+(881.00/70.00))+.3;
  }
  else if (adc <= 741 && adc > 701)
  {
    return  ((-adc/80.00)+(901.00/80.00))+.5;
  }
  else if (adc <= 701 && adc > 522)
  {
    return  ((-adc/358.00)+(798.00/179.00))+.2;
  }
  else if (adc <= 522 && adc >= 347)
  {
   return ((484.528*adc-60503)/(pow(adc,2)-551.626*adc+78197.4))+.25;  
  }
  else return 15;
}
