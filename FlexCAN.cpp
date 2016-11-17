// -------------------------------------------------------------
// a simple Arduino Teensy 3.1/3.2/3.6 CAN driver
// by teachop
// dual CAN support for MK66FX1M0 by Pawelsky
// Interrupt driven Rx/Tx with buffers, object oriented callbacks by Collin Kidder
//
#include "FlexCAN.h"
#include "kinetis_flexcan.h"

#define FLEXCANb_MCR(b)                   (*(vuint32_t*)(b))
#define FLEXCANb_CTRL1(b)                 (*(vuint32_t*)(b+4))
#define FLEXCANb_RXMGMASK(b)              (*(vuint32_t*)(b+0x10))
#define FLEXCANb_IFLAG1(b)                (*(vuint32_t*)(b+0x30))
#define FLEXCANb_IMASK1(b)                (*(vuint32_t*)(b+0x28))
#define FLEXCANb_RXFGMASK(b)              (*(vuint32_t*)(b+0x48))
#define FLEXCANb_MBn_CS(b, n)             (*(vuint32_t*)(b+0x80+n*0x10))
#define FLEXCANb_MBn_ID(b, n)             (*(vuint32_t*)(b+0x84+n*0x10))
#define FLEXCANb_MBn_WORD0(b, n)          (*(vuint32_t*)(b+0x88+n*0x10))
#define FLEXCANb_MBn_WORD1(b, n)          (*(vuint32_t*)(b+0x8C+n*0x10))
#define FLEXCANb_IDFLT_TAB(b, n)          (*(vuint32_t*)(b+0xE0+(n*4)))
#define FLEXCANb_MB_MASK(b, n)            (*(vuint32_t*)(b+0x880+(n*4)))

CAN_filter_t FlexCAN::defaultMask;

// -------------------------------------------------------------
FlexCAN::FlexCAN(uint8_t id)
{
  flexcanBase = FLEXCAN0_BASE;
#ifdef __MK66FX1M0__
  if(id > 0) flexcanBase = FLEXCAN1_BASE;
#endif

  // Default mask is allow everything
  defaultMask.rtr = 0;
  defaultMask.ext = 0;
  defaultMask.id = 0;
  
  rx_buffer_head = 0;
  rx_buffer_tail = 0;
  tx_buffer_head = 0;
  tx_buffer_tail = 0;
  
  for (int i = 0; i < SIZE_LISTENERS; i++) listener[i] = NULL;
}


// -------------------------------------------------------------
void FlexCAN::end(void)
{
  // enter freeze mode
  FLEXCANb_MCR(flexcanBase) |= (FLEXCAN_MCR_HALT);
  while(!(FLEXCANb_MCR(flexcanBase) & FLEXCAN_MCR_FRZ_ACK))
    ;
}


// -------------------------------------------------------------
void FlexCAN::begin(uint32_t baud, const CAN_filter_t &mask, uint8_t txAlt, uint8_t rxAlt)
{
  // set up the pins
  if(flexcanBase == FLEXCAN0_BASE)
  {
    Serial.println("Begin setup of CAN0");
#ifdef __MK66FX1M0__
    //  3=PTA12=CAN0_TX,  4=PTA13=CAN0_RX (default)
    // 29=PTB18=CAN0_TX, 30=PTB19=CAN0_RX (alternative)
    if(txAlt == 1) CORE_PIN29_CONFIG = PORT_PCR_MUX(2); else CORE_PIN3_CONFIG = PORT_PCR_MUX(2); 
    if(rxAlt == 1) CORE_PIN30_CONFIG = PORT_PCR_MUX(2); else CORE_PIN4_CONFIG = PORT_PCR_MUX(2);// | PORT_PCR_PE | PORT_PCR_PS; 
#else   
    //  3=PTA12=CAN0_TX,  4=PTA13=CAN0_RX (default)
    // 32=PTB18=CAN0_TX, 25=PTB19=CAN0_RX (alternative)
    if(txAlt == 1) CORE_PIN32_CONFIG = PORT_PCR_MUX(2); else CORE_PIN3_CONFIG = PORT_PCR_MUX(2); 
    if(rxAlt == 1) CORE_PIN25_CONFIG = PORT_PCR_MUX(2); else CORE_PIN4_CONFIG = PORT_PCR_MUX(2);// | PORT_PCR_PE | PORT_PCR_PS;
#endif
  }
#ifdef __MK66FX1M0__
  else if(flexcanBase == FLEXCAN1_BASE)
  {
      Serial.println("Begin setup of CAN1");
    // 33=PTE24=CAN1_TX, 34=PTE25=CAN1_RX (default)
    // NOTE: Alternative CAN1 pins are not broken out on Teensy 3.6
    CORE_PIN33_CONFIG = PORT_PCR_MUX(2);
    CORE_PIN34_CONFIG = PORT_PCR_MUX(2);// | PORT_PCR_PE | PORT_PCR_PS;
  }
#endif

  // select clock source 16MHz xtal
  OSC0_CR |= OSC_ERCLKEN;
  if(flexcanBase == FLEXCAN0_BASE) SIM_SCGC6 |=  SIM_SCGC6_FLEXCAN0;
#ifdef __MK66FX1M0__
  else if(flexcanBase == FLEXCAN1_BASE) SIM_SCGC3 |=  SIM_SCGC3_FLEXCAN1;
#endif
  FLEXCANb_CTRL1(flexcanBase) &= ~FLEXCAN_CTRL_CLK_SRC;

  // enable CAN
  FLEXCANb_MCR(flexcanBase) |=  FLEXCAN_MCR_FRZ;
  FLEXCANb_MCR(flexcanBase) &= ~FLEXCAN_MCR_MDIS;
  while(FLEXCANb_MCR(flexcanBase) & FLEXCAN_MCR_LPM_ACK)
    ;
  // soft reset
  FLEXCANb_MCR(flexcanBase) ^=  FLEXCAN_MCR_SOFT_RST;
  while(FLEXCANb_MCR(flexcanBase) & FLEXCAN_MCR_SOFT_RST)
    ;
  // wait for freeze ack
  while(!(FLEXCANb_MCR(flexcanBase) & FLEXCAN_MCR_FRZ_ACK))
    ;
  // disable self-reception
  FLEXCANb_MCR(flexcanBase) |= FLEXCAN_MCR_SRX_DIS;

  // segment splits and clock divisor based on baud rate
    if ( 50000 == baud ) {
    FLEXCANb_CTRL1(flexcanBase) = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(1)
                                | FLEXCAN_CTRL_PSEG1(7) | FLEXCAN_CTRL_PSEG2(3) | FLEXCAN_CTRL_PRESDIV(19));
  } else if ( 100000 == baud ) {
    FLEXCANb_CTRL1(flexcanBase) = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(1)
                                | FLEXCAN_CTRL_PSEG1(7) | FLEXCAN_CTRL_PSEG2(3) | FLEXCAN_CTRL_PRESDIV(9));
  } else if ( 250000 == baud ) {
    FLEXCANb_CTRL1(flexcanBase) = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(1)
                                | FLEXCAN_CTRL_PSEG1(7) | FLEXCAN_CTRL_PSEG2(3) | FLEXCAN_CTRL_PRESDIV(3));
  } else if ( 500000 == baud ) {
    FLEXCANb_CTRL1(flexcanBase) = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(1)
                                | FLEXCAN_CTRL_PSEG1(7) | FLEXCAN_CTRL_PSEG2(3) | FLEXCAN_CTRL_PRESDIV(1));
  } else if ( 1000000 == baud ) {
    FLEXCANb_CTRL1(flexcanBase) = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(0)
                                | FLEXCAN_CTRL_PSEG1(1) | FLEXCAN_CTRL_PSEG2(1) | FLEXCAN_CTRL_PRESDIV(1));
  } else { // 125000
    FLEXCANb_CTRL1(flexcanBase) = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(1)
                                | FLEXCAN_CTRL_PSEG1(7) | FLEXCAN_CTRL_PSEG2(3) | FLEXCAN_CTRL_PRESDIV(7));
  }

  FLEXCANb_MCR(flexcanBase) |= FLEXCAN_MCR_IRMQ; //enable per-mailbox filtering
  //now have to set default mask and filter for all the RX mailboxes or they won't receive anything by default.
  CAN_filter_t defaultFilter;
  defaultFilter.ext = 0;
  defaultFilter.rtr = 0;
  defaultFilter.id = 0;
  for (int c = 0; c < NUM_MAILBOXES - numTxMailboxes; c++)
  {
     setMask(0, c);
     setFilter(defaultFilter, c);
  }
    
  // start the CAN
  FLEXCANb_MCR(flexcanBase) &= ~(FLEXCAN_MCR_HALT);
  // wait till exit of freeze mode
  while(FLEXCANb_MCR(flexcanBase) & FLEXCAN_MCR_FRZ_ACK);

  // wait till ready
  while(FLEXCANb_MCR(flexcanBase) & FLEXCAN_MCR_NOT_RDY);
  
  setNumTXBoxes(2);
    
#if defined(__MK20DX256__)
  NVIC_SET_PRIORITY(IRQ_CAN_MESSAGE, IRQ_PRIORITY);
  NVIC_ENABLE_IRQ(IRQ_CAN_MESSAGE);
#elif defined(__MK64FX512__)
  NVIC_SET_PRIORITY(IRQ_CAN0_MESSAGE, IRQ_PRIORITY);
  NVIC_ENABLE_IRQ(IRQ_CAN0_MESSAGE);
#elif defined(__MK66FX1M0__)
  if(flexcanBase == FLEXCAN0_BASE) 
  {
    NVIC_SET_PRIORITY(IRQ_CAN0_MESSAGE, IRQ_PRIORITY);
    NVIC_ENABLE_IRQ(IRQ_CAN0_MESSAGE);
  }
  else
  {
    NVIC_SET_PRIORITY(IRQ_CAN1_MESSAGE, IRQ_PRIORITY);
    NVIC_ENABLE_IRQ(IRQ_CAN1_MESSAGE);      
  }
#endif

  FLEXCANb_IMASK1(flexcanBase) = 0xFFFF; //enable interrupt masks for all 16 mailboxes

  Serial.println("CAN initialized properly");
}


 /* \brief Initializes mailboxes to the requested mix of RX and TX boxes
 *
 * \param txboxes How many of the 8 boxes should be used for TX
 *
 * \retval number of tx boxes set.
 *
 */
int FlexCAN::setNumTXBoxes(int txboxes) {
    int c;

    if (txboxes > 15) txboxes = 15;
    if (txboxes < 1) txboxes = 1;
    numTxMailboxes = txboxes;

    //Inialize RX boxen
    for (c = 0; c < NUM_MAILBOXES - numTxMailboxes; c++) {
        FLEXCANb_MBn_CS(flexcanBase, c) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_RX_EMPTY);
    }

    //Initialize TX boxen
    for (c = NUM_MAILBOXES - numTxMailboxes; c < NUM_MAILBOXES; c++) {
        FLEXCANb_MBn_CS(flexcanBase, c) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
    }
    
    return (numTxMailboxes);
}

 /* \brief Sets a per-mailbox filter. Sets both the storage and the actual mailbox.
 *
 * \param filter is a filled out filter structure, n is the mailbox to update
 *
 * \retval Nothing
 *
 */
void FlexCAN::setFilter(const CAN_filter_t &filter, uint8_t n)
{
   if (n < NUM_MAILBOXES - numTxMailboxes) 
   {
       MBFilters[n] = filter;
       if (filter.ext) 
       {
          FLEXCANb_MBn_ID(flexcanBase, n) = (filter.id & FLEXCAN_MB_ID_EXT_MASK);
          FLEXCANb_MBn_CS(flexcanBase, n) |= FLEXCAN_MB_CS_IDE;
       } else {
          FLEXCANb_MBn_ID(flexcanBase, n) = FLEXCAN_MB_ID_IDSTD(filter.id);
          FLEXCANb_MBn_CS(flexcanBase, n) &= ~FLEXCAN_MB_CS_IDE;
       }
   }
}

/*
 * Per mailbox masks can only be set in freeze mode so have to enter that mode if not already there.
 */
void FlexCAN::setMask(uint32_t mask, uint8_t n)
{
    if (n >= NUM_MAILBOXES - numTxMailboxes) return;
    
    if (!(FLEXCANb_MCR(flexcanBase) & FLEXCAN_MCR_FRZ_ACK)) { //enter freeze mode if not already there
       FLEXCANb_MCR(flexcanBase) |= FLEXCAN_MCR_FRZ;
       FLEXCANb_MCR(flexcanBase) |= FLEXCAN_MCR_HALT;
       while(!(FLEXCANb_MCR(flexcanBase) & FLEXCAN_MCR_FRZ_ACK));
    }
    
    FLEXCANb_MB_MASK(flexcanBase, n) = mask;
    
    //exit freeze mode and wait until it is unfrozen.
    FLEXCANb_MCR(flexcanBase) &= ~FLEXCAN_MCR_HALT;
    while(FLEXCANb_MCR(flexcanBase) & FLEXCAN_MCR_FRZ_ACK);
}


// -------------------------------------------------------------
int FlexCAN::available(void)
{
    int val;
    if (rx_buffer_head != rx_buffer_tail) 
    {
        val = rx_buffer_head - rx_buffer_tail;
        //Now, because this is a cyclic buffer it is possible that the ordering was reversed
        //So, handle that case
        if (val < 0) val += SIZE_RX_BUFFER;
    }
    else return 0;
    
    return val;
}


/**
 * \brief Retrieve a frame from the RX buffer
 *
 * \param buffer Reference to the frame structure to fill out
 *
 * \retval 0 no frames waiting to be received, 1 if a frame was returned
 */
int FlexCAN::read(CAN_message_t &msg)
{
    if (rx_buffer_head == rx_buffer_tail) return 0;
    msg.id = rx_frame_buff[rx_buffer_tail].id;
    msg.ext = rx_frame_buff[rx_buffer_tail].ext;
    msg.len = rx_frame_buff[rx_buffer_tail].len;
    for (int c = 0; c < 8; c++) msg.buf[c] = rx_frame_buff[rx_buffer_tail].buf[c];
    rx_buffer_tail = (rx_buffer_tail + 1) % SIZE_RX_BUFFER;

    return 1;
}

/**
 * \brief Send a frame out of this canbus port
 *
 * \param txFrame The filled out frame structure to use for sending
 *
 * \note Will do one of two things - 1. Send the given frame out of the first available mailbox
 * or 2. queue the frame for sending later via interrupt. Automatically turns on TX interrupt
 * if necessary.
 * 
 * Returns whether sending/queueing succeeded. Will not smash the queue if it gets full.
 */    
int FlexCAN::write(const CAN_message_t &msg)
{
  // find an available buffer
  int buffer = -1;
  for (int index = NUM_MAILBOXES - numTxMailboxes - 1; index < NUM_MAILBOXES; index++) {
    if ((FLEXCANb_MBn_CS(flexcanBase, index) & FLEXCAN_MB_CS_CODE_MASK) == FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE)) {
      buffer = index;
      break;// found one
    }    
  }

  if (buffer > -1)
  {
     Serial.println("Writing a frame directly.");
     writeTxRegisters(msg, buffer);
     return 1;
  }
  else //no mailboxes available. Try to buffer it
  {
    uint8_t temp;
    temp = (tx_buffer_tail + 1) % SIZE_TX_BUFFER;
    if (temp == tx_buffer_head) return 0;
    tx_frame_buff[tx_buffer_tail].id = msg.id;
    tx_frame_buff[tx_buffer_tail].ext = msg.ext;
    tx_frame_buff[tx_buffer_tail].len  = msg.len;
    for (int c = 0; c < 8; c++) tx_frame_buff[tx_buffer_tail].buf[c] = msg.buf[c];
    tx_buffer_tail = temp;
    return 1;
      
  }

  return 0; //could not send the frame!
}

void FlexCAN::writeTxRegisters(const CAN_message_t &msg, uint8_t buffer)
{
  // transmit the frame
  FLEXCANb_MBn_CS(flexcanBase, buffer) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
  if(msg.ext) {
    FLEXCANb_MBn_ID(flexcanBase, buffer) = (msg.id & FLEXCAN_MB_ID_EXT_MASK);
  } else {
    FLEXCANb_MBn_ID(flexcanBase, buffer) = FLEXCAN_MB_ID_IDSTD(msg.id);
  }
  FLEXCANb_MBn_WORD0(flexcanBase, buffer) = (msg.buf[0]<<24)|(msg.buf[1]<<16)|(msg.buf[2]<<8)|msg.buf[3];
  FLEXCANb_MBn_WORD1(flexcanBase, buffer) = (msg.buf[4]<<24)|(msg.buf[5]<<16)|(msg.buf[6]<<8)|msg.buf[7];
  if(msg.ext) {
    FLEXCANb_MBn_CS(flexcanBase, buffer) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
                                         | FLEXCAN_MB_CS_LENGTH(msg.len) | FLEXCAN_MB_CS_SRR | FLEXCAN_MB_CS_IDE;
  } else {
    FLEXCANb_MBn_CS(flexcanBase, buffer) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
                                         | FLEXCAN_MB_CS_LENGTH(msg.len);
  }
}

void FlexCAN::readRxRegisters(CAN_message_t& msg, uint8_t buffer)
{
  // get identifier and dlc
  msg.len = FLEXCAN_get_length(FLEXCANb_MBn_CS(flexcanBase, buffer));
  msg.ext = (FLEXCANb_MBn_CS(flexcanBase, buffer) & FLEXCAN_MB_CS_IDE)? 1:0;
  msg.id  = (FLEXCANb_MBn_ID(flexcanBase, buffer) & FLEXCAN_MB_ID_EXT_MASK);
  if(!msg.ext) {
    msg.id >>= FLEXCAN_MB_ID_STD_BIT_NO;
  }

  // copy out message
  uint32_t dataIn = FLEXCANb_MBn_WORD0(flexcanBase, buffer);
  msg.buf[3] = dataIn;
  dataIn >>=8;
  msg.buf[2] = dataIn;
  dataIn >>=8;
  msg.buf[1] = dataIn;
  dataIn >>=8;
  msg.buf[0] = dataIn;
  if ( 4 < msg.len ) {
    dataIn = FLEXCANb_MBn_WORD1(flexcanBase, buffer);
    msg.buf[7] = dataIn;
    dataIn >>=8;
    msg.buf[6] = dataIn;
    dataIn >>=8;
    msg.buf[5] = dataIn;
    dataIn >>=8;
    msg.buf[4] = dataIn;
  }
  for( int loop=msg.len; loop<8; ++loop ) {
    msg.buf[loop] = 0;
  }
}


//a message either came in or was freshly sent. Figure out which and act accordingly.
void FlexCAN::message_isr(void) 
{
    uint8_t temp;
    uint32_t status = FLEXCANb_IFLAG1(flexcanBase);
    FLEXCANb_IFLAG1(flexcanBase) = status; //writing its value back to itself clears all flags
    for (int i = 0; i < 16; i++) if (status & (1 << i)) //has this mailbox triggered an interrupt?
    {
        uint32_t code = FLEXCAN_get_code(FLEXCANb_MBn_CS(flexcanBase, i));
        switch (code)
        {
        /* //these codes exist but aren't useful here as far as I know. Just kept for reference and in case they're needed some day.
        case 0: //inactive Receive box. Must be a false alarm!?
            break;
        case 1: //mailbox is busy. Don't touch it.
            break;
        case 4: //rx empty already. Why did it interrupt then?            
            break;                                 
        case 9: //TX being aborted.
            break;
        case 0xA: //remote request response. Remote request is deprecated and I don't care about it. Be gone!
            break; 
        case 0xC: //TX mailbox is full and will be sent as soon as possible
            break;
        case 0xE: //remote request junk again. Go away.
            break;
        */    
        case 2: //rx full, that's more like it. Copy the frame to RX buffer
        case 6: //rx overrun. We didn't get there in time and a second frame tried to enter the MB. Whoops... Can probably still grab the frame though.
            temp = (rx_buffer_head + 1) % SIZE_RX_BUFFER;
            if (temp != rx_buffer_tail) 
            {
                readRxRegisters((CAN_message_t &)rx_frame_buff[rx_buffer_head], i);
                rx_buffer_head = temp;
            }
            break;
        case 8: //TX inactive. Just chillin' waiting for a message to send. Let's see if we've got one.
            if (tx_buffer_head != tx_buffer_tail) 
            { //if there is a frame in the queue to send
                writeTxRegisters((CAN_message_t &)tx_frame_buff[tx_buffer_head], i);                
                tx_buffer_head = (tx_buffer_head + 1) % SIZE_TX_BUFFER;
            }
            break;
        }
    }
}

boolean FlexCAN::attachObj(CANListener *listener)
{
    for (int i = 0; i < SIZE_LISTENERS; i++)
    {
        if (this->listener[i] == NULL)
        {
            this->listener[i] = listener;
            listener->callbacksActive = 0;
            return true;            
        }
    }
    return false;
}

boolean FlexCAN::detachObj(CANListener *listener)
{
    for (int i = 0; i < SIZE_LISTENERS; i++)
    {
        if (this->listener[i] == listener)
        {
            this->listener[i] = NULL;           
            return true;            
        }
    }
    return false;  
}

void FlexCAN::bus_off_isr(void)
{
    
}

void FlexCAN::error_isr(void)
{
    
}

void FlexCAN::tx_warn_isr(void)
{
    
}

void FlexCAN::rx_warn_isr(void)
{
    
}

void FlexCAN::wakeup_isr(void)
{
    
}

void can0_message_isr(void) {
    Can0.message_isr();
}

void can0_bus_off_isr(void) {
    Can0.bus_off_isr();
}

void can0_error_isr(void) {
    Can0.error_isr();
}

void can0_tx_warn_isr(void) {
    Can0.tx_warn_isr();
}

void can0_rx_warn_isr(void) {
    Can0.rx_warn_isr();
}

void can0_wakeup_isr(void) {
    Can0.wakeup_isr();
}

void can1_message_isr(void) {
    Can1.message_isr();
}

void can1_bus_off_isr(void) {
    Can1.bus_off_isr();
}

void can1_error_isr(void) {
    Can1.error_isr();
}

void can1_tx_warn_isr(void) {
    Can1.tx_warn_isr();
}

void can1_rx_warn_isr(void) {
    Can1.rx_warn_isr();
}

void can1_wakeup_isr(void) {
    Can1.wakeup_isr();
}


CANListener::CANListener()
{
    callbacksActive = 0; //none. Bitfield were bits 0-15 are the mailboxes and bit 31 is the general callback
}

//an empty version so that the linker doesn't complain that no implementation exists.
void CANListener::gotFrame(CAN_message_t &frame, int mailbox)
{
  
}

void CANListener::attachMBHandler(uint8_t mailBox)
{
    if (mailBox >= 0 && mailBox < NUM_MAILBOXES)
    {
        callbacksActive |= (1L << mailBox);
    }
}

void CANListener::detachMBHandler(uint8_t mailBox)
{
    if (mailBox >= 0 && mailBox < NUM_MAILBOXES)
    {
        callbacksActive &= ~(1L << mailBox);
    }  
}

void CANListener::attachGeneralHandler()
{
    callbacksActive |= (1L << 31);
}

void CANListener::detachGeneralHandler()
{
    callbacksActive &= ~(1L << 31);
}

FlexCAN Can0(0);
#ifdef __MK66FX1M0__
FlexCAN Can1(1);
#endif