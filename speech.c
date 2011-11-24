#include <math.h>
#include <SDL/SDL.h>

#ifdef USE_SAM
#include "include/sam/sam.h"
#else
#include "include/espeak/speak_lib.h"
#endif 

struct speech_list_t {
	char voice[ 256 ];
	char data[ 256 ];
	struct speech_list_t *next;
};

typedef struct speech_list_t speech_list_t;

// SAM linked-list and mutex
static speech_list_t *speech_first = NULL;
static speech_list_t *speech_last = NULL;
static  SDL_mutex    *speech_mx;

// SPEECH buffers
static uint8_t        buf[ 2 * 10 * 44100 ];
#ifndef USE_SAM
static uint8_t       *p_buf;
#endif 
static int            buf_size;
static int            buf_pos;
static int            speaking = 0;

// Visualization
static unsigned char vis_buffer[ 160 ];
static unsigned char vis_mul[ 160 ];
static int  do_vis = 0;
static int ctr = 0;

static int opened;

int speech_vis( unsigned char **buffer ) {
	do_vis = 1;
	if( speaking == 1 ) {
		*buffer = vis_buffer;
		return( 0 );
	}
	return( -1 );
}

static void sdl_mixer( void *unused, Uint8 *stream, int stream_len ) {
	int i;
	float f,dc;
	int len=stream_len;
	if( buf_pos >= buf_size ) {
    if( speaking == 1 ) speaking = 2;
	} else {
#ifdef USE_SAM
		if( ( buf_size - buf_pos ) < len ) len = buf_size - buf_pos;
		for( i = 0; i < len; i++ ) {
			stream[ i ] = buf[ buf_pos++ ];
		  buf_pos++;
		}
		if( len < stream_len ) memset( &stream[ len ], 128, stream_len - len );
#else		
		if( ( buf_size - buf_pos ) < len ) len = buf_size - buf_pos;
		memcpy( stream, &buf[ buf_pos ], len );
		buf_pos += len;
		if( len < stream_len ) memset( &stream[ len ], 0, stream_len - len );
#endif
		
		if( do_vis ) {
			f = stream[ 0 ];
			dc = f;
			vis_buffer[ 0 ] = ( ( ( int )( f - dc ) ) * vis_mul[ i ] ) >> 8;
		  if( len > 160 ) len = 160;
			for( i = 1; i < len; i++ ) {
				f += ( ( ( float )stream[ i << 1 ] ) - f ) / 10;
				dc += ( f - dc ) / 20;
				vis_buffer[ i ] = ( ( ( int )( f - dc ) ) * vis_mul[ i ] ) >> 8;
			}
			for( ; i < 160; i++ ) {
				f += ( ( ( float )0 ) - f ) / 10;
				dc += ( f - dc ) / 20;
				vis_buffer[ i ] = ( ( ( int )( f - dc ) ) * vis_mul[ i ] ) >> 8;
			}
		}
	}

}


#ifndef USE_SAM
int espeak_cb( short *pcm_s16, int samples, espeak_EVENT *e ) {
  if( samples > 0 ) {
    if( p_buf == buf ) buf_size = 0;
    buf_pos = 0;
    samples *= sizeof( short );
    if( samples > &buf[ sizeof( buf ) ] - p_buf ) samples = &buf[ sizeof( buf ) ] - p_buf;
    memcpy( p_buf, pcm_s16, samples );
    p_buf += samples;
    buf_size += samples;
  }
  return( 0 );
}
#endif

void speech_open() {
  int n;
  float m;
  speech_mx = SDL_CreateMutex();
  SDL_AudioSpec fmt;

#ifndef USE_SAM
  int freq;
  freq = espeak_Initialize( AUDIO_OUTPUT_SYNCHRONOUS, 10000, ".", 0 );
  printf( "Speech [info]: eSpeak frequency: %i\n", freq );
  espeak_SetSynthCallback( espeak_cb );
#endif 

#ifdef USE_SAM
  fmt.freq     = 11025;
  fmt.format   = AUDIO_U8;
#else
  fmt.freq     = freq;
  fmt.format   = AUDIO_S16;
#endif   
  fmt.channels = 1;
  fmt.samples  = 1024;
  fmt.callback = sdl_mixer;
  fmt.userdata = NULL;
  for( n = 0; n < 160; n++ ) {
    vis_mul[ n ] = ( unsigned char )( ( 1.0 - cos( ( ( float )n ) / 80.0 * 3.1415f ) ) * 127 );
  }
  if ( SDL_OpenAudio( &fmt, NULL ) < 0 ) {
    fprintf( stderr, "Speech [error]: Unable to open SDL audio\n" );
  } else {
    printf( "Speech [info]: Initialized\n" );
    opened = 1;
  }

}

static void buf_play() {
#ifdef USE_SAM	
	buf_size    /= SAM_SCALE;
	buf_size = ( buf_size > 512 ? buf_size - 512 : 0 );
#endif
  buf_pos      = 0;
  speaking = 1;
	SDL_PauseAudio( 0 );
}

uint8_t speech_poll() {
  speech_list_t *p_list;
	if( speaking == 2 ) {
    SDL_PauseAudio( 1 );
    speaking = 0;
	}
	if( speaking == 0 ) {
    SDL_mutexP( speech_mx );
		if( speech_first ) {
  		// Queue pop
      p_list = speech_first;
      speech_first = p_list->next;
      SDL_mutexV( speech_mx );
#ifdef USE_SAM
  		// Set parameters
  		sam_params( SAM_SCALE, SAM_SING, SAM_SPEED, SAM_PITCH, SAM_MOUTH, SAM_THROAT );
      buf_pos = 0;
  		buf_size = sizeof( buf );
  		if( sam_speak( buf, &buf_size, p_list->data ) == 0 ) buf_play();
#else

      printf( "Speech [info]: eSpeak voice return: %i\n", espeak_SetVoiceByName( p_list->voice ) );
      espeak_SetParameter(espeakRATE,130,0);
	    espeak_SetParameter(espeakRANGE,0,0);
//		espeak_SetParameter(espeakVOLUME,volume,0);
//		espeak_SetParameter(espeakPITCH,pitch,0);
//		espeak_SetParameter(espeakCAPITALS,option_capitals,0);
//		espeak_SetParameter(espeakPUNCTUATION,option_punctuation,0);
//		espeak_SetParameter(espeakWORDGAP,wordgap,0);
//		espeak_SetParameter(espeakLINELENGTH,option_linelength,0);
//		espeak_SetPunctuationList(option_punctlist);

      p_buf = buf;
      espeak_Synth( p_list->data, strlen( p_list->data ) + 1, 0, POS_CHARACTER, 0, 4352, NULL, NULL );
      buf_play();
#endif
  		free( p_list );
		} else {
      SDL_mutexV( speech_mx );
		}
	}
	return( speaking );
}

void speech_queue( char* voice, char* speak ) {
  speech_list_t *p_list = malloc( sizeof( speech_list_t ) );

  if( !opened ) return;
    
  if( p_list == NULL ) {
    printf( "Speech [error]: Out of memory" );
    return;
  }

  strcpy( p_list->voice, voice );
#ifdef USE_SAM
  // Copy and convert to phenomes
  memset( p_list->data, 0, 256 );
  strcpy( p_list->data, speak );
  sam_phenomes( p_list->data );
  strcat( p_list->data, " \x9b\0" ); // TODO: is this really necessary?
#else
  strcpy( p_list->data, speak );
#endif

 	// Queue push
  SDL_mutexP( speech_mx );
	p_list->next = NULL;
	if( speech_first ) {
		speech_last->next = p_list;
	} else {
		speech_first = p_list;
	}  
	speech_last = p_list;
  SDL_mutexV( speech_mx );
	
	speech_poll();

}

void speech_free() {
  SDL_DestroyMutex( speech_mx );
}

int speech_state() {
	return( speaking );
}
