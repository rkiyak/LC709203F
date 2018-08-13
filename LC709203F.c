/*
  @
  @   Date               :        13.08.2018 / Monday
  @
  @   Contact            :        Editing by Muhammet Rasit KIYAK                    
                                  @https://www.linkedin.com/in/mrstkyk/
  @                               mrstkyk@gmail.com 
  @
  @   Description        :        This Library for LC709203F Gauge Sensor
  @
*/


/* 
   @Brief         CRC Checker
   @Description   Needed byte for Cyclic Redundancy Check.
                  DeviceID, Control Reg, LSB Byte, MSB Byte, CRC Byte 
   @Parameter     uint8_t *rec_values -> array name for check
                  uint8_t len         -> lengt of array
   @Return value  uint8_t
 */
 uint8_t check_crc(uint8_t *rec_values, uint8_t len)
{
  uint8_t crc = 0x00                                                    ;    
  uint8_t current_byte                                                  ;    
  uint8_t bit                                                           ;    
  for (current_byte = 0 ; current_byte < len ; current_byte++) 
  {
   crc ^= (rec_values[current_byte])                                    ;    
    for (bit = 8 ; bit > 0 ;bit--) 
    {
     if (crc & 0x80) 
       {
         crc = (crc << 1) ^ cmd_CRCpolynomial                           ;                      
       }
     else 
       {
         crc = (crc << 1)                                               ;    
       }
    }
  }
  return crc                                                            ;    
}


/* 
   @Brief         Calibration before the startup
   @Description   Obtain OCV(Open Circuit Voltage) in 10ms.
                  Figure 8 on datasheet
   @Parameter     I2C_HandleTypeDef   -> i2c 
   @Return value  None
 */
void LC709203_RSOC_before(I2C_HandleTypeDef i2c)
{
  static uint8_t buf[4]                                                 ;                            
  buf[0]=cmd_beforeRSOC                                                 ;      
  buf[1]=0x55                                                           ;    
  buf[2]=0xAA                                                           ;    
  buf[3]=check_crc(buf,3)                                               ;          
  HAL_I2C_Master_Transmit(&i2c,DeviceID<<1,buf,3,20)                    ;    
  HAL_Delay(10)                                                         ;     //Need 10ms for calibrating
}


/* @
   @Brief           Thermistor set
   @Description     Sets B-constant of the thermistor to be measured. Refer to
                    the specification sheet of the thermistor for the set value to use
   @Parameter       I2C_HandleTypeDef   -> i2c 
   @Return value    uint16_t
 */
uint16_t LC709203_thermistorB(I2C_HandleTypeDef i2c)
{
  static uint8_t buf[4]                                                 ;                                
  buf[0]=cmd_thermistorB                                                ;      
  buf[1]=0x34                                                           ;    
  buf[2]=0x0D                                                           ;    
  buf[3]=check_crc(buf,3)                                               ;                                                                                  
  HAL_I2C_Master_Transmit(&i2c,DeviceID<<1,buf,3,20)                    ;    
  HAL_Delay(1)                                                          ;    
  HAL_I2C_Master_Receive(&i2c,DeviceID<<1,buf,3,20)                     ;     
  return ((buf[1]<<8)|(buf[0]))                                         ;    
}


/*  
   @Brief           Automatic Initialization RSOC
   @Description     Calibration when start-up.More efficiency than RSOC_before @Figure 7 on datasheet
   @Parameter       I2C_HandleTypeDef -> i2c 
   @Return value    None
 */
void LC709203_RSOC_initial(I2C_HandleTypeDef i2c)
{
  static uint8_t buf[4]                                                 ;                            
  buf[0]=cmd_initialRSOC                                                ;      
  buf[1]=0x55                                                           ;    
  buf[2]=0xAA                                                           ;    
  buf[3]=check_crc(buf,3)                                               ;                                     
  HAL_I2C_Master_Transmit(&i2c,DeviceID<<1,buf,3,20)                    ;    
  HAL_Delay(10)                                                         ;     //Need 10ms for calibrating
}


/*  
   @Brief           Displays Cell Temperature
   @Description     This register contains the cell temperature from -20C (0×09E4) to
                    +60 C (0×0D04) measured in 0.1C units
   @Parameter       I2C_HandleTypeDef -> i2c
   @Return value    float
 */
float LC709203_cellTemperature(I2C_HandleTypeDef i2c)
{
  static uint8_t buf[4]                                                 ;                            
  buf[0]=cmd_cellTemperature                                            ;      
  buf[1]=0xA6                                                           ;    
  buf[2]=0x0B                                                           ;    
  buf[3]=check_crc(buf,3)                                               ;                                           
  HAL_I2C_Master_Transmit(&i2c,DeviceID<<1,buf,3,20)                    ;    
  HAL_Delay(1)                                                          ;        
  HAL_I2C_Master_Receive(&i2c,DeviceID<<1,buf,3,20)                     ;     
  int TempSum =((buf[1]<<8)|(buf[0]))                                   ;    
  float temp=(TempSum - 2371.5)                                         ;    
  return temp                                                           ;    
}


/*  
   @Brief           Displays Cell Voltage
   @Description     This register contains the voltage on VDD 1 mV units
   @Parameter       I2C_HandleTypeDef -> i2c
   @Return value    uint16_t
 */
uint16_t LC709203_cellVoltage(I2C_HandleTypeDef i2c) 
{
  static uint8_t buf[3]                                                 ;    
  buf[0]=cmd_cellVoltage                                                ;    
  HAL_I2C_Master_Transmit(&i2c,DeviceID<<1,buf,0,20)                    ;    
  HAL_Delay(5)                                                          ;    
  HAL_I2C_Master_Receive(&i2c,DeviceID<<1,buf,3,20)                     ;     
  return ((buf[1]<<8)|(buf[0]))                                         ;    
}


/*  
   @Brief           This register is used to control the reporting of RSOC
   @Description     In Auto mode the RSOC is reported as it increases or decreases.
                    In Charge mode the RSOC is not permitted to decrease. In
                    Discharge mode the RSOC is not permitted to increase
   @Parameter       I2C_HandleTypeDef -> i2c
   @Return value    uint16_t
 */
uint16_t LC709203_currentDirection(I2C_HandleTypeDef i2c)
{
  static uint8_t buf[4]                                                 ;                                 
  buf[0]=cmd_currentDirection                                           ;      
  buf[1]=0x00                                                           ;    
  buf[2]=0x00                                                           ;    
  buf[3]=check_crc(buf,3)                                               ;                                           
  HAL_I2C_Master_Transmit(&i2c,DeviceID<<1,buf,3,20)                    ;    
  HAL_Delay(1)                                                          ;          
  HAL_I2C_Master_Receive(&i2c,DeviceID<<1,buf,3,20)                     ;     
  return ((buf[1]<<8)|(buf[0]))                                         ;    
}


/*  
   @Brief           This register contains the adjustment value for a battery
                    type to improve the RSOC precision
   @Description     The deeper adjustment of APA may
                    improve the accuracy  
   @Parameter       I2C_HandleTypeDef -> i2c
   @Return value    uint16_t
 */
uint16_t LC709203_adjusmentPackApplicaiton(I2C_HandleTypeDef i2c)
{
  static uint8_t buf[4]                                                 ;                                
  buf[0]=cmd_APA                                                        ;      
  buf[1]=0x00                                                           ;    
  buf[2]=0x00                                                           ;    
  buf[3]=check_crc(buf,3)                                               ;                                 
  HAL_I2C_Master_Transmit(&i2c,DeviceID<<1,buf,3,20)                    ;    
  HAL_Delay(1)                                                          ;        
  HAL_I2C_Master_Receive(&i2c,DeviceID<<1,buf,3,20)                     ;     
  return ((buf[1]<<8)|(buf[0]))                                         ;    
}


/*  
   @Brief           This is used to compensate for the delay of the thermistor
                    measurement caused by a capacitor across the thermistor
   @Description     The default value has been found to meet most of circuits
                    where a capacitor like showing in Figure 13 is not put
   @Parameter       I2C_HandleTypeDef -> i2c
   @Return value    uint16_t
 */
uint16_t LC709203_adjusmentPackThermistor(I2C_HandleTypeDef i2c)
{
  static uint8_t buf[4]                                                 ;                               
  buf[0]=cmd_APT                                                        ;      
  buf[1]=0x1E                                                           ;    
  buf[2]=0x00                                                           ;    
  buf[3]=check_crc(buf,3)                                               ;                                           
  HAL_I2C_Master_Transmit(&i2c,DeviceID<<1,buf,3,20)                    ;    
  HAL_Delay(1)                                                          ;         
  HAL_I2C_Master_Receive(&i2c,DeviceID<<1,buf,3,20)                     ;     
  return ((buf[1]<<8)|(buf[0]))                                         ;    
}


/*  
   @Brief           RSOC is reported in 1% units over the range 0% to 100%
   @Description   
   @Parameter       I2C_HandleTypeDef -> i2c
   @Return value    uint16_t
 */
uint16_t LC709203_RSOC(I2C_HandleTypeDef i2c)
{
  static uint8_t buf[3]                                                 ;    
  buf[0]=cmd_RSOC                                                       ;    
  HAL_I2C_Master_Transmit(&i2c,DeviceID<<1,buf,0,20)                    ;    
  HAL_Delay(5)                                                          ;    
  HAL_I2C_Master_Receive(&i2c,DeviceID<<1,buf,3,20)                     ;     
  return ((buf[1]<<8)|(buf[0]))                                         ;    
}


/*  
   @Brief           This is the same as RSOC with a resolution of 0.1% over
                    the range 0.0% to 100.0%
   @Description   
   @Parameter       I2C_HandleTypeDef -> i2c
   @Return value    uint16_t
 */
uint16_t LC709203_indicatorToEmpty(I2C_HandleTypeDef i2c)
{
  static uint8_t buf[3]                                                 ;    
  buf[0]=cmd_ITE                                                        ;    
  HAL_I2C_Master_Transmit(&i2c,DeviceID<<1,buf,0,20)                    ;    
  HAL_Delay(5)                                                          ;    
  HAL_I2C_Master_Receive(&i2c,DeviceID<<1,buf,3,20)                     ;     
  return ((buf[1]<<8)|(buf[0]))                                         ;    
}


/*  
   @Brief           This is an ID number of an LSI
   @Description   
   @Parameter       I2C_HandleTypeDef -> i2c
   @Return value    uint16_t
 */
uint16_t LC709203_IC_Version(I2C_HandleTypeDef i2c)
{
  static uint8_t buf[3]                                                 ;    
  buf[0]=cmd_ICversion                                                  ;    
  HAL_I2C_Master_Transmit(&i2c,DeviceID<<1,buf,0,20)                    ;                      
  HAL_Delay(5)                                                          ;    
  HAL_I2C_Master_Receive(&i2c,DeviceID<<1,buf,3,20)                     ;     
  return ((buf[1]<<8)|(buf[0]))                                         ;    
}


/*  
   @Brief           This register is used to select the battery profile to
                    be used
   @Description   
   @Parameter       I2C_HandleTypeDef -> i2c
   @Return value    uint16_t
 */
uint16_t LC709203_changeOfTheParameter(I2C_HandleTypeDef i2c)
{
  static uint8_t buf[4]                                                 ;                                  
  buf[0]=cmd_changeOftheParameter                                       ;  
  buf[1]=0x00                                                           ;                                                             
  buf[2]=0x00                                                           ;                                                             
  buf[3]=check_crc(buf,3)                                               ;                                                                                
  HAL_I2C_Master_Transmit(&i2c,DeviceID<<1,buf,3,20)                    ;           
  HAL_Delay(1)                                                          ;               
  HAL_I2C_Master_Receive(&i2c,DeviceID<<1,buf,3,20)                     ;            
  return ((buf[1]<<8)|(buf[0]))                                         ;                      
}


/*  
   @Brief           The ALARMB pin will be set low when the RSOC value
                    falls below this value, will be released from low when RSOC
                    value rises than this value
   @Description   
   @Parameter       I2C_HandleTypeDef -> i2c
   @Return value    uint16_t
 */
uint16_t LC709203_RSOC_alarmLow(I2C_HandleTypeDef i2c)
{
  static uint8_t buf[4]                                                 ;                               
  buf[0]=cmd_alarmLowRSOC                                               ;          
  buf[1]=0x08                                                           ;                                                             
  buf[2]=0x00                                                           ;                                                             
  buf[3]=check_crc(buf,3)                                               ;                                                                                    
  HAL_I2C_Master_Transmit(&i2c,DeviceID<<1,buf,3,20)                    ;           
  HAL_Delay(1)                                                          ;               
  HAL_I2C_Master_Receive(&i2c,DeviceID<<1,buf,3,20)                     ;            
  return ((buf[1]<<8)|(buf[0]))                                         ;                      
}


/*  
   @Brief           The ALARMB pin will be set low if VDD falls below this
                    value, will be released from low if VDD rises than this value
   @Description   
   @Parameter       I2C_HandleTypeDef -> i2c
   @Return value    uint16_t
 */
uint16_t LC709203_alarmLowCellVoltage(I2C_HandleTypeDef i2c)
{
  static uint8_t buf[4]                                                 ;                                     
  buf[0]=cmd_alarmLowCellVoltage                                        ;   
  buf[1]=0x00                                                           ;                                                             
  buf[2]=0x00                                                           ;                                                             
  buf[3]=check_crc(buf,3)                                               ;                                                                                   
  HAL_I2C_Master_Transmit(&i2c,DeviceID<<1,buf,3,20)                    ;           
  HAL_Delay(1)                                                          ;              
  HAL_I2C_Master_Receive(&i2c,DeviceID<<1,buf,3,20)                     ;            
  return ((buf[1]<<8)|(buf[0]))                                         ;                      
}


/*  
   @Brief           The LSI has two power modes. Sleep (0x15 = 02) or
                    Operational mode (0x15 = 01)
   @Description   
   @Parameter       I2C_HandleTypeDef -> i2c
   @Return value    uint16_t
 */
uint16_t LC709203_IC_powerMode(I2C_HandleTypeDef i2c)
{
  static uint8_t buf[4]                                                 ;                             
  buf[0]=cmd_ICpowerMode                                                ;           
  buf[1]=0x00                                                           ;                                                             
  buf[2]=0x00                                                           ;                                                             
  buf[3]=check_crc(buf,3)                                               ;                                                                                     
  HAL_I2C_Master_Transmit(&i2c,DeviceID<<1,buf,3,20)                    ;           
  HAL_Delay(1)                                                          ;               
  HAL_I2C_Master_Receive(&i2c,DeviceID<<1,buf,3,20)                     ;            
  return ((buf[1]<<8)|(buf[0]))                                         ;                      
}


/*  
   @Brief           This selects the Thermistor mode
   @Description   
   @Parameter       I2C_HandleTypeDef -> i2c
   @Return value    uint16_t
 */
uint16_t LC709203_statusBit(I2C_HandleTypeDef i2c)
{
  static uint8_t buf[4]                                                 ;                                    
  buf[0]=cmd_statusBit                                                  ;             
  buf[1]=0x00                                                           ;                                                             
  buf[2]=0x00                                                           ;                                                             
  buf[3]=check_crc(buf,3)                                               ;                                                                              
  HAL_I2C_Master_Transmit(&i2c,DeviceID<<1,buf,3,20)                    ;           
  HAL_Delay(1)                                                          ;               
  HAL_I2C_Master_Receive(&i2c,DeviceID<<1,buf,3,20)                     ;            
  return ((buf[1]<<8)|(buf[0]))                                         ;                      
}


/*  
   @Brief           The LSI contains a data file comprised of two battery
                    profiles. This register contains identity of the data file.
   @Description   
   @Parameter       I2C_HandleTypeDef -> i2c
   @Return value    uint16_t
 */
uint16_t LC709203_numberOfTheParameter(I2C_HandleTypeDef i2c)
{
  static uint8_t buf[3]                                                 ;                                                   
  buf[0]=cmd_numberOfTheParameter                                       ;                                         
  HAL_I2C_Master_Transmit(&i2c,DeviceID<<1,buf,0,20)                    ;                      
  HAL_Delay(1)                                                          ;                                                            
  HAL_I2C_Master_Receive(&i2c,DeviceID<<1,buf,3,20)                     ;            
  return ((buf[1]<<8)|(buf[0]))                                         ;                      
}
