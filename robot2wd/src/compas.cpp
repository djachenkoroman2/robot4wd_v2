#include "Arduino.h"
#include "compas.h"
#include <MechaQMC5883.h> //библиотека QMC5883 для работы с датчиком QMC583, но не с HMC5883
MechaQMC5883 qmc; //создаем объект для работы с датчиком (магнитометром), даем ему имя qmc


void SetupCompas()
{
  Wire.begin(); //старт связи по протоколу I2C 
  qmc.init(); //инициализация датчика QMC5883
}

int GetHeading()
{
  int x,y,z;
  qmc.read(&x,&y,&z); //считываем значения X, Y и Z с датчика 
  
  int heading=atan2(x, y)/0.0174532925; // рассчитываем направление в градусах используя значения X и Y  
 //преобразуем результат в диапазон от 0 до 360
  if(heading < 0) 
  heading+=360;
  heading = 360-heading;

  return heading;
}