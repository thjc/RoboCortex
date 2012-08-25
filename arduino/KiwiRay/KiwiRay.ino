#include <Wire.h>

// Frequency gen. interrupt speed (reload value)
#define SPD 128

// Motor driver step size outputs
#define DS0 13
#define DS1 12
#define DS2 11

// Motor drivers enable output
#define DEN 10

// Wheels direction & step outputs
#define W0D  8
#define W0S  9
#define W1D  6
#define W1S  7
#define W2D  3
#define W2S  4

// Servo PWM output
#define SPW  5 // Must be 5

// Readability
#define STEPSIZE( n ) digitalWrite( DS0, n & 1 ); \
                      digitalWrite( DS1, n & 2 ); \
                      digitalWrite( DS2, n & 4 );
#define ENABLE( b )   digitalWrite( DEN, !(b) );

// Math
#define ABS( x ) ( x & 0x8000 ? ( ~x ) + 1 : x )

struct raw_ctrl
{
    int16_t motor[3];
    uint8_t camera;
    uint8_t stepsize;
} __attribute__(( packed ));

struct xyr_ctrl
{
    int8_t X;
    int8_t Y;
    int8_t R;
    uint8_t camera;
    uint8_t stepsize;
} __attribute__(( packed ));

struct pos_response
{
    uint8_t start;
    uint8_t addr;
    int16_t motor[3];
} __attribute__(( packed ));

typedef union data_buffer
{
    uint8_t raw_data[64];
    struct raw_ctrl raw;
    struct xyr_ctrl xyr;
    struct pos_response pos;
} data_buffer;

// Communications handling
uint8_t state, addr, len, got;

data_buffer buf;


// Frequency generation for the wheels 
int16_t       wheel[ 3 ] = { 0, 0, 0 };
int32_t       whacc[ 3 ] = { 0, 0, 0 };
uint16_t      whext[ 3 ];

// Connection loss protection
uint32_t timer_data = 1;

// Power save on idle
uint32_t timer_zero = 0;

ISR( TIMER2_OVF_vect ) {
  TCNT2 = SPD;

  digitalWrite( W0D, ( wheel[ 0 ] & 0x8000 ) != 0 );
  digitalWrite( W1D, ( wheel[ 1 ] & 0x8000 ) != 0 );
  digitalWrite( W2D, ( wheel[ 2 ] & 0x8000 ) != 0 );
  whext[ 0 ] += ABS( wheel[ 0 ] );
  whext[ 1 ] += ABS( wheel[ 1 ] );
  whext[ 2 ] += ABS( wheel[ 2 ] );
  whacc[ 0 ] += wheel[ 0 ];
  whacc[ 1 ] += wheel[ 1 ];
  whacc[ 2 ] += wheel[ 2 ];
  digitalWrite( W0S, ( whext[ 0 ] & 0x8000 ) != 0 );
  digitalWrite( W1S, ( whext[ 1 ] & 0x8000 ) != 0 );
  digitalWrite( W2S, ( whext[ 2 ] & 0x8000 ) != 0 );
  // Connection loss and idle detection
  if( wheel[ 0 ] || wheel[ 1 ] || wheel[ 2 ] ) timer_zero = 10000; 
  if( timer_data ) timer_data--;
  if( timer_zero ) timer_zero--;
  ENABLE( timer_data && timer_zero );
}   

void setup() {
  // Pin setup
  pinMode( W0D, OUTPUT );
  pinMode( W0S, OUTPUT );
  pinMode( W1D, OUTPUT );
  pinMode( W1S, OUTPUT );
  pinMode( W2D, OUTPUT );
  pinMode( W2S, OUTPUT );
  pinMode( DS0, OUTPUT );
  pinMode( DS1, OUTPUT );
  pinMode( DS2, OUTPUT );
  pinMode( DEN, OUTPUT );
  pinMode( SPW, OUTPUT );
  
  // Configure Servo PWM on T0
  analogWrite( SPW, 96 );
  TCCR0A |= 0b00000011;
  TCCR0B  = 0b11001100;
  OCR0A   = 255;
  OCR0B   = 110;

  // Configure frequency gen. interrupt
  TIMSK2 &= ~( ( 1 << TOIE2  ) );
  TCCR2A &= ~( ( 1 << WGM21  ) | ( 1 << WGM20 ) );
  TCCR2B &= ~( ( 1 << WGM22  ) );
  ASSR   &= ~( ( 1 << AS2    ) );
  TIMSK2 &= ~( ( 1 << OCIE2A ) );   
  TCCR2B &= ~( ( 1 << CS22   ) | ( 1 << CS21  ) | ( 1 << CS20 ) );
  //TCCR2B |=  ( ( 1 << CS20 ) ); // Prescaler 1:1
  TCCR2B |=  ( ( 1 << CS21 ) ); // Prescaler 1:8
  //TCCR2B |=  ( ( 1 << CS21 ) | ( 1 << CS20 ) ); // Prescaler 1:32
  TCNT2   = SPD;
  TIMSK2 |=  ( ( 1 << TOIE2  ) );
    
  // Initialize I2C
  Wire.begin();
  // Set silly slow I2C for PIC soft I2C
  //_SFR_BYTE( TWSR ) |= _BV( TWPS0 ) | _BV( TWPS1 );

  // Initialize PC-communications
  Serial.begin( 115200 );
  
  // Initialize motor control
  STEPSIZE( 6 );
  ENABLE( 0 );
}

void process() {
  if( addr & 0xFC ) {
    // start i2c
    Wire.beginTransmission( addr );
    Wire.write( buf.raw_data, len );
    Wire.endTransmission();
    sei();
  } else {
    cli();
    if( addr & 0x1 ) {
      // Addr 0x01: direct 16-bit motor control
      wheel[ 0 ] = buf.raw.motor[0];
      wheel[ 1 ] = buf.raw.motor[1];
      wheel[ 2 ] = buf.raw.motor[2];
      OCR0B = ( ( ( (uint16_t)buf.raw.camera ) * 86 ) >> 8 ) + 48;
      STEPSIZE( buf.raw.stepsize );
    } else if( ! addr & 0x1 ) {
      // Addr 0x00: 8-bit XYR control
      wheel[ 0 ] = ( buf.xyr.X + buf.xyr.R ) * 128;
      wheel[ 1 ] = -64 * buf.xyr.X + 110 * buf.xyr.Y + buf.xyr.R * 128;
      wheel[ 2 ] = -64 * buf.xyr.X - 110 * buf.xyr.Y + buf.xyr.R * 128;
      OCR0B = ( ( ( (uint16_t) buf.xyr.camera ) * 86 ) >> 8 ) + 48;
      STEPSIZE( buf.xyr.stepsize );
    } 
    sei();
    // report accumulated wheel ticks
    if ( addr & 0x2 )
    {
      buf.pos.start = 0xFF;
      buf.pos.addr = addr;
      buf.pos.motor[0] = (int16_t) (whacc[0] >> 16);
      buf.pos.motor[1] = (int16_t) (whacc[1] >> 16);
      buf.pos.motor[2] = (int16_t) (whacc[2] >> 16);
      Serial.write(buf.raw_data, 8);
    }
  }
}

void loop() {
  unsigned char data;

  if( Serial.available() ) {
    data = Serial.read();
    switch( state ) {
      case 0: // wait for start
        if( data == 0xFF ) state = 1;
        break;
      case 1: // address
        addr = data;
        if( addr & 0xFC ) {
          state = 2;
        } else {
          state = 3;
          switch (addr & 0x3) {
            case 1:
            case 3:
              len = 8;
              break;
            default:  
              len = 5;
          }
          timer_data = 10000;
        }
        got = 0;
        break;
      case 2: // Length & R/W bit
        len = ( data & 0x3F ) + 1;
        // TODO: implement read
        state = 3;
        break;
      case 3: // data
        buf.raw_data[ got++ ] = data;
        if( got == len ) {
          process();
          state = 0;
        }
        break;
    }
  }
}
