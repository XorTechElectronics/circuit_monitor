
#define I2C_SLAVE_ADDRESS 0x30 // Address of the slave

#include <Arduino.h>

//MegaTinyCore
//0   PA6   Digital OUT : Enable Relay    Also UART TX
//1   PA7   Analog : Current
//2   PA1   SDA
//3   PA2   SCL
//4   PA3   Analog : Voltage
//5   PA0   UPDI

volatile  uint8_t   irq_data;

          uint8_t   irq_counter   = 0;

          uint16_t  raw_current   =  34;  //0x0022
          uint16_t  raw_voltage   = 679;  //0x02A7

volatile  uint8_t   reg_array[]   = {0x25, 0x50, 0x75, 0x10};

// ************************************************************************************************
// ************************************************************************************************
//  I2C Slave functions
// ************************************************************************************************
// ************************************************************************************************
void I2C_init(uint8_t address)
{
  //From Datasheet
    //To enable the TWI as slave, write the Slave Address (ADDR) in TWIn.SADDR, and write a '1' to 
    // the ENABLE bit in the Slave Control A register (TWIn.SCTRLA). The TWI peripheral will wait to 
    //receive a byte addressed to it.
  cli();

  // load address into SADDR address register
    TWI0_SADDR    = I2C_SLAVE_ADDRESS << 1;
  //Write '1' to ENABLE bit and Data Interrupt Enable
    TWI0_SCTRLA   = (1<<TWI_DIEN_bp) | (1<<TWI_APIEN_bp) | (1<<TWI_PIEN_bp) | (1<<TWI_ENABLE_bp);

  sei();
}



// ************************************************************************************************
// ************************************************************************************************
//  Interrupts
// ************************************************************************************************
// ************************************************************************************************
ISR( TWI0_TWIS_vect )
{
  // Response dependant on TWI0_SSTATUS register
  //  7     6     5       4     3     2       1     0   
  //  DIF   APIF  CLKHOLD RXACK COLL  BUSERR  DIR   AP
  //
    if(TWI0_SSTATUS & TWI_APIF_bm)					      //Address match/stop interrupt
    {
        if (TWI0_SSTATUS & TWI_COLL_bm)
        {
            TWI0_SSTATUS |= TWI_COLL_bm;			    //Clear Collision flag
            TWI0_SCTRLB = TWI_SCMD_COMPTRANS_gc;	//complete transaction
            return;
        }
        if(TWI0_SSTATUS & TWI_AP_bm)
            TWI0_SCTRLB = TWI_SCMD_RESPONSE_gc;		//Send ACK after address match
        else
            TWI0_SCTRLB = TWI_SCMD_COMPTRANS_gc;	//complete transaction after Stop

        irq_counter = 0;
    }
 

    if(TWI0_SSTATUS & TWI_DIF_bm)					        //Data interrupt
    {
        if(TWI0_SSTATUS & TWI_DIR_bm)
        {            
            //Send Data
              TWI0.SDATA  = reg_array[irq_counter];

            //next in reg_array
              irq_counter = irq_counter + 1;            

            //Send Response
              TWI0_SCTRLB = TWI_SCMD_RESPONSE_gc;
        }
        else
        {
            TWI0_SCTRLB = TWI_SCMD_RESPONSE_gc;
            irq_data = TWI0.SDATA;						    //Receive data written by Master
        }
    }
}


// ************************************************************************************************
// ************************************************************************************************
//  SETUP
// ************************************************************************************************
// ************************************************************************************************
void setup() {

  Serial.begin(115200);
  Serial.println("I2C Slave Starting .... ");

  // init I2C Slave 
    I2C_init(I2C_SLAVE_ADDRESS);

}
// ************************************************************************************************
// ************************************************************************************************
//  LOOP
// ************************************************************************************************
// ************************************************************************************************
void loop() {

  raw_voltage = analogRead(PIN_A3);
  raw_current = analogRead(PIN_A7);

  reg_array[0]  = raw_voltage >> 4;
  reg_array[1]  = raw_voltage & 0x0F;   //why!!!! //I2C Master doesnt seem to like top bit of data set so jiggle it around here to prevent it
  reg_array[2]  = raw_current >> 4;
  reg_array[3]  = raw_current & 0x0F;   //why!!!! //I2C Master doesnt seem to like top bit of data set so jiggle it around here to prevent it

  Serial.print("raw_v=");
  Serial.print(raw_voltage);
  Serial.print("\t\traw_i=");
  Serial.print(raw_current); 
  Serial.print("\t\treg_array=");
  Serial.print(reg_array[0],HEX);
  Serial.print("\t");
  Serial.print(reg_array[1],HEX);
  Serial.print("\t");
  Serial.print(reg_array[2],HEX);
  Serial.print("\t");
  Serial.print(reg_array[3],HEX);    
  Serial.println("");

  delay(1000);
}