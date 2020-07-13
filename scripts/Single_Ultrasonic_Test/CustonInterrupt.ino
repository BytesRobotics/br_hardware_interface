#include "Arduino.h"
#include "wiring_private.h"

extern voidFuncPtr ISRcallback[EXTERNAL_NUM_INTERRUPTS];
extern uint32_t    ISRlist[EXTERNAL_NUM_INTERRUPTS];
extern uint32_t    nints; // Stores total number of attached interrupts

static void __initialize()
{
  memset(ISRlist,     0, sizeof(ISRlist));
  memset(ISRcallback, 0, sizeof(ISRcallback));
  nints = 0;

  NVIC_DisableIRQ(EIC_IRQn);
  NVIC_ClearPendingIRQ(EIC_IRQn);
  NVIC_SetPriority(EIC_IRQn, 0);
  NVIC_EnableIRQ(EIC_IRQn);

  // Enable GCLK for IEC (External Interrupt Controller)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_EIC));

/* Shall we do that?
  // Do a software reset on EIC
  EIC->CTRL.SWRST.bit = 1 ;
  while ((EIC->CTRL.SWRST.bit == 1) && (EIC->STATUS.SYNCBUSY.bit == 1)) { }
*/

  // Enable EIC
  EIC->CTRL.bit.ENABLE = 1;
  while (EIC->STATUS.bit.SYNCBUSY == 1) { }
}

int pinPeripheral( uint32_t ulPin, uint32_t ulPort, EPioType ulPeripheral )
{
  switch ( ulPeripheral )
  {
    case PIO_DIGITAL:
    case PIO_INPUT:
    case PIO_INPUT_PULLUP:
    case PIO_OUTPUT:
      // Disable peripheral muxing, done in pinMode
//      PORT->Group[g_APinDescription[ulPin].ulPort].PINCFG[g_APinDescription[ulPin].ulPin].bit.PMUXEN = 0 ;

      // Configure pin mode, if requested
      if ( ulPeripheral == PIO_INPUT )
      {
        pinMode( ulPin, INPUT ) ;
      }
      else
      {
        if ( ulPeripheral == PIO_INPUT_PULLUP )
        {
          pinMode( ulPin, INPUT_PULLUP ) ;
        }
        else
        {
          if ( ulPeripheral == PIO_OUTPUT )
          {
            pinMode( ulPin, OUTPUT ) ;
          }
          else
          {
            // PIO_DIGITAL, do we have to do something as all cases are covered?
          }
        }
      }
    break ;

    case PIO_ANALOG:
    case PIO_SERCOM:
    case PIO_SERCOM_ALT:
    case PIO_TIMER:
    case PIO_TIMER_ALT:
    case PIO_EXTINT:
    case PIO_COM:
    case PIO_AC_CLK:
#if 0
      // Is the pio pin in the lower 16 ones?
      // The WRCONFIG register allows update of only 16 pin max out of 32
         SerialUSB.println("2.1");
      if ( ulPin < 16 )
      {
           SerialUSB.println("2.2");

        PORT->Group[ulPort].WRCONFIG.reg = PORT_WRCONFIG_WRPMUX | PORT_WRCONFIG_PMUXEN | PORT_WRCONFIG_PMUX( ulPeripheral ) |
                                                                    PORT_WRCONFIG_WRPINCFG |
                                                                    PORT_WRCONFIG_PINMASK( ulPin ) ;
      }
      else
      {
           SerialUSB.println("2.3");

        PORT->Group[ulPort].WRCONFIG.reg = PORT_WRCONFIG_HWSEL |
                                                                    PORT_WRCONFIG_WRPMUX | PORT_WRCONFIG_PMUXEN | PORT_WRCONFIG_PMUX( ulPeripheral ) |
                                                                    PORT_WRCONFIG_WRPINCFG |
                                                                    PORT_WRCONFIG_PINMASK( ulPin - 16 ) ;
      }
#else
      if ( ulPin & 1 ) // is pin odd?
      {
           SerialUSB.println("2.4");

        uint32_t temp ;

        // Get whole current setup for both odd and even pins and remove odd one
        temp = (PORT->Group[ulPort].PMUX[ulPin >> 1].reg) & PORT_PMUX_PMUXE( 0xF ) ;
        // Set new muxing
        PORT->Group[ulPort].PMUX[ulPin >> 1].reg = temp|PORT_PMUX_PMUXO( ulPeripheral ) ;
        // Enable port mux
        PORT->Group[ulPort].PINCFG[ulPin].reg |= PORT_PINCFG_PMUXEN ;
      }
      else // even pin
      {
           SerialUSB.println("2.5");

        uint32_t temp ;

           SerialUSB.println("2.5.1");

        temp = (PORT->Group[ulPort].PMUX[ulPin >> 1].reg) & PORT_PMUX_PMUXO( 0xF ) ;
           SerialUSB.println("2.5.2");

           Serial.println(ulPin);
           Serial.println(ulPort);
           
           SerialUSB.println(PORT->Group[ulPort].PMUX[ulPin >> 1].reg, BIN);
           SerialUSB.println( temp|PORT_PMUX_PMUXE( ulPeripheral ) , BIN);

        PORT->Group[ulPort].PMUX[ulPin >> 1].reg = temp|PORT_PMUX_PMUXE( ulPeripheral ) ;
           SerialUSB.println("2.5.3");

        PORT->Group[ulPort].PINCFG[ulPin].reg |= PORT_PINCFG_PMUXEN ; // Enable port mux
      }
#endif
    break ;

       SerialUSB.println("2.6");


    case PIO_NOT_A_PIN:
      return -1 ;
    break ;
  }

     SerialUSB.println("2.7");


  return 0 ;
}

void attachInterrupt(uint32_t pin, uint32_t port, uint32_t extint, voidFuncPtr callback, uint32_t mode)
{
  static int enabled = 0;
  uint32_t config;
  uint32_t pos;
  
  EExt_Interrupts in = EExt_Interrupts(extint);


  if (!enabled) {
    __initialize();
    enabled = 1;
  }
  Serial.print("nints");
  SerialUSB.println(nints);
   SerialUSB.println("1");

  // Enable wakeup capability on pin in case being used during sleep
  uint32_t inMask = 1 << in;
  EIC->WAKEUP.reg |= inMask;
  
   SerialUSB.println("2");

  // Assign pin to EIC
  pinPeripheral(pin, port, PIO_EXTINT);

  
   SerialUSB.println("3");

  // Only store when there is really an ISR to call.
  // This allow for calling attachInterrupt(pin, NULL, mode), we set up all needed register
  // but won't service the interrupt, this way we also don't need to check it inside the ISR.
  if (callback)
  {
    // Store interrupts to service in order of when they were attached
    // to allow for first come first serve handler
    uint32_t current = 0;

    // Check if we already have this interrupt
    for (current=0; current<nints; current++) {
      if (ISRlist[current] == inMask) {
        break;
      }
    }
    if (current == nints) {
      // Need to make a new entry
      nints++;
    }
    ISRlist[current] = inMask;       // List of interrupt in order of when they were attached
    ISRcallback[current] = callback; // List of callback adresses

    // Look for right CONFIG register to be addressed
    if (in > EXTERNAL_INT_7) {
      config = 1;
      pos = (in - 8) << 2;
    } else {
      config = 0;
      pos = in << 2;
    }

    // Configure the interrupt mode
    EIC->CONFIG[config].reg &=~ (EIC_CONFIG_SENSE0_Msk << pos); // Reset sense mode, important when changing trigger mode during runtime
    switch (mode)
    {
      case LOW:
        EIC->CONFIG[config].reg |= EIC_CONFIG_SENSE0_LOW_Val << pos;
        break;

      case HIGH:
        EIC->CONFIG[config].reg |= EIC_CONFIG_SENSE0_HIGH_Val << pos;
        break;

      case CHANGE:
        EIC->CONFIG[config].reg |= EIC_CONFIG_SENSE0_BOTH_Val << pos;
        break;

      case FALLING:
        EIC->CONFIG[config].reg |= EIC_CONFIG_SENSE0_FALL_Val << pos;
        break;

      case RISING:
        EIC->CONFIG[config].reg |= EIC_CONFIG_SENSE0_RISE_Val << pos;
        break;
    }
  }
  
   SerialUSB.println("4");
  // Enable the interrupt
  EIC->INTENSET.reg = EIC_INTENSET_EXTINT(inMask);
  
   SerialUSB.println("5");
}
