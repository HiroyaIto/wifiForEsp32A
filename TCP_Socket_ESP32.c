// ********** TCP Socket Client for ESP32 **********
// 
// 概要:
//  センサからデータを取得し、TCP over IPv6 over Wi-Fiでデータを送信する。
// 
// 
// コンパイルコマンド（ESP-IDF）
// > make clean
// > make flash monitor

// for spi 2020.04.03
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

#include "pretty_effect.h"

// for gpio 2020.03.30
#include "freertos/queue.h"
#include "driver/gpio.h"


//#include "string.h"
//#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_event_loop.h"
//#include "esp_sntp.h"
#include "lwip/sockets.h"
//#include "driver/i2c.h"

#include "freertos/semphr.h" // 
#include "lwip/dns.h" //
#include "lwip/netdb.h" // 
#include "lwip/igmp.h" //
#include "esp_event.h" //
#include "nvs_flash.h" //
#include "soc/rtc_periph.h" //
#include "driver/spi_slave.h" //
#include "esp_spi_flash.h" //

// timer
#include "esp_types.h"//
#include "soc/timer_group_struct.h"//
#include "driver/periph_ctrl.h"//
#include "driver/timer.h"//
#include "esp_timer.h"//


#define GPIO_HANDSHAKE 2
#define GPIO_MOSI 12
#define GPIO_MISO 13
#define GPIO_SCLK 15
#define GPIO_CS 14

#ifdef CONFIG_IDF_TARGET_ESP32
#define RCV_HOST    HSPI_HOST
#define DMA_CHAN    2

#elif defined CONFIG_IDF_TARGET_ESP32S2
#define RCV_HOST    SPI2_HOST
#define DMA_CHAN    RCV_HOST

#endif


#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK 18 
#define PIN_NUM_CS  5 

//#define PIN_NUM_DC   21
//#define PIN_NUM_RST  18
//#define PIN_NUM_BCKL 5

#define GPIO_OUTPUT_IO_0    21
#define GPIO_OUTPUT_IO_1    22
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1))
#define GPIO_INPUT_IO_0     4
//#define GPIO_INPUT_IO_1     5
#define GPIO_INPUT_IO_1     2
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))
#define ESP_INTR_FLAG_DEFAULT 0

#ifndef HIGH
    #define HIGH 1
#endif
#ifndef LOW
    #define LOW 0
#endif

#define PARALLEL_LINES 16
// for spi end 2020.04.03

#include <time.h>   // 2020.02.29



#define MODENO_SERVER  1  // 動作モード識別番号（1＝サーバ）
#define MODENO_CLIENT  2  // 動作モード識別番号（2＝クライアント）

#define I2C_ACK  0x0      // I2C ACK（確認応答）
#define I2C_NAK  0x1      // I2C NAK（否定応答）

// INA226 レジスタ情報
//#define INA226_ADDR               0x40  // INA226 I2C Address (A0=A1=GND)
#define KX224_ADDR               0x1E  // KX224 I2C Address (A0=GND)
#define INA226_REG_CONFIGURATION  0x00
#define INA226_REG_CALIBRATION    0x05
#define INA226_REG_BUS_VOLTAGE    0x02  // Bus Voltage (ROM)
#define INA226_REG_CURRENT        0x04  // Current (ROM)
#define INA226_REG_POWER          0x03  // Power (ROM)

//#define LENGTH_ringBuffer_dataFromSensor  1024  // [octet]  // 2020.02.28
#define LENGTH_ringBuffer_dataFromSensor  65536  // [octet]  // 2020.02.28
// センサから取得したデータ格納用のリングバッファの長さ
// センサから取得する単位データ長の整数倍であること
// データは1単位8octetなので、128単位格納できる
//#define LENGTH_linearBuffer_dataToSocket  1024  // [octet]  // 2020.02.28
#define LENGTH_linearBuffer_dataToSocket  65536  // [octet]  // 2020.02.28
// TCP Socketへの送信データ格納用の線形バッファの長さ
// バッファに溜め込む想定のデータ量に対して十分に長いこと
// このプログラムは80単位ごとにTCP Socketで送信する

// for KX224 2020.02.26
// register address
#define KX224_XOUT_L              (0x06)  // 標準
//#define KX224_XOUT_L              (0x18)   // 連続データリードテスト
#define KX224_WHO_AM_I            (0x0F)
#define KX224_CNTL1               (0x18)
#define KX224_ODCNTL              (0x1B)
// register data
#define KX224_WAI_VAL             (0x2B)
#define KX224_CNTL1_GSEL_8G       (0x00)
#define KX224_CNTL1_GSEL_16G      (0x08)
#define KX224_CNTL1_GSEL_32G      (0x10)
#define KX224_CNTL1_RES           (1 << 6)
#define KX224_CNTL1_VAL           (KX224_CNTL1_RES | KX224_CNTL1_GSEL_8G)
#define KX224_ODCNTL_OSA_50HZ     (15)      //Output Data Rate 25600Hz
#define KX224_ODCNTL_VAL          (KX224_ODCNTL_OSA_50HZ)
#define KX224_CNTL1_GSELMASK      (0x18)
#define KX224_CNTL1_PC1           (1 << 7)

// define spi address 2020.04.03
#define WHO_AM_I 0x0F
#define CNTL1 0x18
#define ODCNTL 0x1B
#define BUF_CNTL1 0x3A
#define BUF_CLEAR 0x3E
#define INC1 0x1C
#define INC4 0x1F
#define BUF_CNTL2 0x3B
#define BUF_STATUS_1 0x3C
#define BUF_STATUS_2 0x3D
#define BUF_READ 0x3F
#define XOUT_L 0x06
#define XOUT_H 0x07
#define YOUT_L 0x08
#define YOUT_H 0x09
#define ZOUT_L 0x0A
#define ZOUT_H 0x0B
#define INS2 0x13

//#define TRANS_NUM 10
//#define TRANS_NUM 100
//#define TRANS_NUM 500
//#define TRANS_NUM 1000
#define TRANS_NUM 2000



uint8_t readByte(spi_device_handle_t spi, unsigned char adr);
uint16_t writeByte(spi_device_handle_t spi, unsigned char adr, unsigned char w_data);
uint16_t read_KX224_340(spi_device_handle_t spi);
void init_TCP_Client( void );

void getData_fromKX224_2( uint8_t *adr );

int transferData_ToSocket( void );
uint16_t write_read_Byte(spi_device_handle_t spi);
uint8_t writeByte2(spi_device_handle_t spi, unsigned char adr, unsigned char data);

void kx224_wifi(void);

int RecieveData_FromMaster( void );

spi_device_handle_t spi;

static unsigned short _g_sens;
int for_ipv4 = 0;  // when for_ipv4=1,indicate IPV4 detected
int data_ready = 0;  // when data_KX224ready=1,indicate interrupt occurerd from KX224

static const char logTag_ESP32_Event[]           = "ESP32_Event";
static const char logTag_init_systemTime[]       = "init_systemTime";
static const char logTag_init_TCP_Client[]       = "init_TCP_Client";
static const char logTag_getData_fromKX224[]    = "getData_fromKX224";
static const char logTag_transferData_ToSocket[] = "transferData_ToSocket";

static clock_t start,end;   // 2020.02.29 for counting time
static int starti,endi;    // 2020.02.29
static uint8_t startil,startiu;
static uint8_t endil,endiu;
static uint8_t _g_sensl,_g_sensu;
static int getData_count;     // 2020.02.29 for identifing first line and end line of transfering contents

static uint8_t Data_count=0; // 2020.04.06
static int amount_temp = 0;  // 2020.04.09 buf_status_1,2の蓄積

static uint8_t mem_flag=0;     //0:linearBuffer_dataToSocket 1:linearBuffer_dataToSocket2

static const int modeNo = MODENO_CLIENT;

//static char string_IPv6addr_destination[ INET6_ADDRSTRLEN ] = "***.***.***.**";  // mobile hot spot ITO-PC2-7566
//static char string_IPv6addr_destination[ INET6_ADDRSTRLEN ] = "***.***.***.*";  // wimax  // let's note
static char string_IPv6addr_destination[ INET6_ADDRSTRLEN ] = "***.***.***.*";  // wimax  // let's note
//static char string_IPv6addr_destination[ INET6_ADDRSTRLEN ] = "***.***.*.*";  // ctc-g-bb4ab6



static const int  TCPport_destination = 61367;

// for gpio interrupt
static xQueueHandle gpio_evt_queue = NULL;

uint8_t rawtemp[6*340+1]={0x00}; // バッファmaxは340サンプル分とする
int read_num;  // BUF_STATUS_1,2で読み込んだ読むべきバッファ数を格納する。

//uint8_t rawtemp[611]={0x00};
//uint8_t rawtemp[611]={0x00};
//uint8_t rawtemp[51]={0x00};
//uint8_t rawtemp[265]={0x00};

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
    //gpio_set_level(GPIO_OUTPUT_IO_1, 1);  // for debug
}



static const wifi_country_t wifi_country = 
{
  .cc = "JP",
  .schan = 1,
  .nchan = 14,
  .max_tx_power = 23,
  .policy = WIFI_COUNTRY_POLICY_MANUAL,
};

wifi_config_t wifi_config = 
{
  .sta = 
  {
	// モバイルスポット
    //.ssid     = "*******",
    //.password = "*******",
    
    // Wimax
    .ssid     = "***********",
    .password = "***********",
    
	//自宅wifi  2.5GならOK. ESP32 WROOMは2.5Gのみ対応の為。
    //.ssid     = "*********",  // OK  2.5G
    ////.ssid     = "**********",    // NG  5G
    //.password = "***********",    
  },
};

#if 0
i2c_config_t i2c_config = {
  .mode = I2C_MODE_MASTER,
  //.sda_io_num = 33,
  //.scl_io_num = 32,
  .sda_io_num = 21,
  .scl_io_num = 22,  
  .sda_pullup_en = GPIO_PULLUP_DISABLE,
  .scl_pullup_en = GPIO_PULLUP_DISABLE,
//.master.clk_speed = 100000,
  .master.clk_speed = 1000000
};


i2c_cmd_handle_t i2cCommandQueue = { 0x00 };
#endif




// ********** 変数定義 **********

char string_IPv4addr_DHCP [  INET_ADDRSTRLEN ] = { 0x00 };
char string_IPv6addr_SLAAC[ INET6_ADDRSTRLEN ] = { 0x00 };

time_t bin_systemTime = 0;
struct tm tm_systemTime = { 0 };
char string_systemTime[ 48 ] = { 0x00 };

//struct sockaddr_in6 portInfo_destination = { 0x00 };  // 宛先ポート情報（クライアントモード）

struct sockaddr_in portInfo_destination = { 0x00 };  // 宛先ポート情報（クライアントモード）

int socket_connect_clientSide = -1;
int result_connect_clientSide = -1;
int length_sendData = -1;     // Socketへ送信しようとするデータサイズ（クライアントモード）
int length_sentData = -1;     // 実際に送信できたデータサイズ（クライアントモード）

uint16_t count_sampleNo = 0;

char ringBuffer_dataFromSensor[ LENGTH_ringBuffer_dataFromSensor ] = { 0x00 };  // センサから取得したデータ格納用のリングバッファ
char linearBuffer_dataToSocket[ LENGTH_linearBuffer_dataToSocket ] = { 0x00 };  // Socketへ送信するデータ格納用の線形バッファ

char rx_buffer[128];
char send_chk_to_master[8] = {0xFE};

char linearBuffer_dataToSocket2[ LENGTH_linearBuffer_dataToSocket ] = { 0x00 };  // Socketへ送信するデータ格納用の線形バッファ ダブルバッファ用 //2020.06.25

unsigned int ringPointer_nextInputStartIndex  = 0;  // リングバッファへデータを入力するとき、次に書き込むべき先頭の配列Index
unsigned int ringPointer_nextOutputStartIndex = 0;  // リングバッファからの出力するとき、次に読み込むべき先頭の配列Index





// SNTPによる時刻合わせ（インターネット接続の確認も兼ねている）
/*     2020.02.26
void init_systemTime( void )
{
  // Timezoneを設定
  setenv( "TZ", "JST-9", 1 );
  tzset();
  
  // SNTP動作モードを設定
  sntp_setoperatingmode( SNTP_OPMODE_POLL );
  sntp_setservername( 0, "ntp.nict.jp" );
  sntp_set_sync_mode( SNTP_SYNC_MODE_IMMED );
  sntp_init();
  
  // SNTPで時刻取得待ち
  while( SNTP_SYNC_STATUS_COMPLETED != sntp_get_sync_status() )
  {
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
  ESP_LOGI( logTag_init_systemTime, "Waiting for SNTP" );
  }
  
  // System Timeを表示
  time( &bin_systemTime );
  localtime_r( &bin_systemTime, &tm_systemTime );
  strftime( string_systemTime, sizeof( string_systemTime ), "%F(%a) %T %z(%Z)", &tm_systemTime );
  ESP_LOGI( logTag_init_systemTime, "System time adjusted by SNTP: %s", string_systemTime );
}
*/


// ********** TCP Socketを初期化 **********
void init_TCP_Client( void )
{
  //  (Step. 5) (Client Mode Only) サーバとの接続用Socketを作成
  
  if( MODENO_CLIENT == modeNo )
  {
    // サーバとの接続用Socketを作成
    //socket_connect_clientSide = socket( AF_INET6, SOCK_STREAM, IPPROTO_TCP );
    socket_connect_clientSide = socket( AF_INET, SOCK_STREAM, IPPROTO_TCP );
    // 引数1: (int)af      : (23)AF_INET6     // IPv6
    // 引数2: (int)type    : ( 1)SOCK_STREAM  // TCP/IP
    // 引数3: (int)protocol: ( 6)IPPROTO_TCP  // TCP
    
    if( -1 == socket_connect_clientSide )
    {
      ESP_LOGI( logTag_init_TCP_Client, "Step. 5: Failed to create socket for connect to server." );
      ESP_LOGI( logTag_init_TCP_Client, "socket_connect_clientSide = %d", socket_connect_clientSide );
      vTaskDelay( 1000 / portTICK_PERIOD_MS );
      abort();
    }
    else
    {
      ESP_LOGI( logTag_init_TCP_Client, "Step. 5: Success to create socket for connect to server." );
    }
  }
  
  
  int flg = 0; // connect fail flag
  // (Step. 6) (Client Mode Only) サーバへ接続
  
  if( MODENO_CLIENT == modeNo )
  {
    //portInfo_destination.sin6_family   = AF_INET6;
    //portInfo_destination.sin6_port     = htons( TCPport_destination );
    //portInfo_destination.sin6_flowinfo = 0;
    //portInfo_destination.sin6_scope_id = 0;
    
    portInfo_destination.sin_family   = AF_INET;
    portInfo_destination.sin_port     = htons( TCPport_destination );
    //portInfo_destination.sin6_flowinfo = 0;
    //portInfo_destination.sin6_scope_id = 0;

    //inet_pton( AF_INET6, string_IPv6addr_destination, &(portInfo_destination.sin6_addr) );
    inet_pton( AF_INET, string_IPv6addr_destination, &(portInfo_destination.sin_addr) );
    
    // サーバへ接続要求
    ESP_LOGI( logTag_init_TCP_Client, "Step. 6: now connect phase." );
    result_connect_clientSide = connect( socket_connect_clientSide, (struct sockaddr *)&portInfo_destination, sizeof( portInfo_destination ) );
    ESP_LOGI( logTag_init_TCP_Client, "Step. 6: now after connect phase." );
    // 返り値: (int): 正常終了なら0
    // 引数1: (SOCKET)socket_connect_clientSide
    // 引数2: (struct sockaddr *)接続先のIPアドレス, TCPポート情報
    // 引数3: (int)namelen: sizeof( addr )
    
    if( -1 == result_connect_clientSide )
    {
      ESP_LOGI( logTag_init_TCP_Client, "Step. 6: Failed to connect to server." );
      ESP_LOGI( logTag_init_TCP_Client, "result_connect_clientSide = %d", result_connect_clientSide );
      vTaskDelay( 1000 / portTICK_PERIOD_MS );
      //abort();  // remove comment out 
      flg = 1;      
    }
    else
    {
      ESP_LOGI( logTag_init_TCP_Client, "Step. 6: Success to connect to server." );
    }
    
    if(flg){   // retry

    inet_pton( AF_INET, string_IPv6addr_destination, &(portInfo_destination.sin_addr) );
    
    // サーバへ接続要求
    result_connect_clientSide = connect( socket_connect_clientSide, (struct sockaddr *)&portInfo_destination, sizeof( portInfo_destination ) );
    // 返り値: (int): 正常終了なら0
    // 引数1: (SOCKET)socket_connect_clientSide
    // 引数2: (struct sockaddr *)接続先のIPアドレス, TCPポート情報
    // 引数3: (int)namelen: sizeof( addr )
    
    if( -1 == result_connect_clientSide )
    {
      ESP_LOGI( logTag_init_TCP_Client, "Step. 6: Failed to connect to server." );
      ESP_LOGI( logTag_init_TCP_Client, "result_connect_clientSide = %d", result_connect_clientSide );
      vTaskDelay( 1000 / portTICK_PERIOD_MS );
      //abort();
      //restart();
      //flg = 1;
    }
    else
    {
	  gpio_set_level(GPIO_OUTPUT_IO_1, 1);
      ESP_LOGI( logTag_init_TCP_Client, "Step. 6: Success to connect to server." );
    }

	}    
    
    // TCP接続確立
    //inet_ntop( AF_INET6, &( portInfo_destination.sin6_addr ), string_IPv6addr_destination, sizeof( string_IPv6addr_destination ) );
    inet_ntop( AF_INET, &( portInfo_destination.sin_addr ), string_IPv6addr_destination, sizeof( string_IPv6addr_destination ) );
    ESP_LOGI( logTag_init_TCP_Client, "******************************" );
    ESP_LOGI( logTag_init_TCP_Client, "* TCP Connection Established *" );
    ESP_LOGI( logTag_init_TCP_Client, "******************************" );
    //ESP_LOGI( logTag_init_TCP_Client, "Connected to: [%s]:%d\n", string_IPv6addr_destination, ntohs( portInfo_destination.sin6_port ) );
    ESP_LOGI( logTag_init_TCP_Client, "Connected to: [%s]:%d\n", string_IPv6addr_destination, ntohs( portInfo_destination.sin_port ) );

  }
}




uint8_t *cr_data;  // pointer for cr

uint16_t count_temp = 0;

uint8_t head_include =0; // syncコード8バイトがあることを示す。

// センサからデータ取得
void getData_fromKX224_2( uint8_t *adr )
{
  uint8_t data_sampleNo_MSByte = 0;
  uint8_t data_sampleNo_LSByte = 0;
  
  uint8_t sync_d = 0xff;
  

  uint16_t i = 0;
  uint16_t temp = 0;
  
  uint8_t crd=0x0d;
  cr_data = &crd;

    // add sync as 8 byte zero 2020.06.22
    // 2001ｘ8byteをメモリに格納。
    //if (head_include == 0) {
    
	
	if (mem_flag == 0){
    	for (i=0;i<8;i++)
  			memcpy( linearBuffer_dataToSocket + i, &sync_d, sizeof( sync_d ) );
  			// 今までの2000サンプル分のメモリ格納処理

		for(i=1;i<=TRANS_NUM;i++){
  			temp = count_temp;
  
  			data_sampleNo_MSByte = (uint8_t)( ( temp >> 8 ) & 0x00FF );
		  	data_sampleNo_LSByte = (uint8_t)( ( temp >> 0 ) & 0x00FF );
  
			memcpy( linearBuffer_dataToSocket + i*8+0, &data_sampleNo_LSByte, sizeof( data_sampleNo_LSByte ) );
	  		memcpy( linearBuffer_dataToSocket + i*8+1, &data_sampleNo_MSByte, sizeof( data_sampleNo_MSByte ) );
			memcpy( linearBuffer_dataToSocket + i*8+2, adr+(i-1)*6+0,        sizeof( uint8_t        ) );
  			memcpy( linearBuffer_dataToSocket + i*8+3, adr+(i-1)*6+1,        sizeof( uint8_t        ) );
  			memcpy( linearBuffer_dataToSocket + i*8+4, adr+(i-1)*6+2,        sizeof( uint8_t        ) );
		  	memcpy( linearBuffer_dataToSocket + i*8+5, adr+(i-1)*6+3,        sizeof( uint8_t        ) );
			memcpy( linearBuffer_dataToSocket + i*8+6, adr+(i-1)*6+4,        sizeof( uint8_t        ) );
	  		memcpy( linearBuffer_dataToSocket + i*8+7, adr+(i-1)*6+5,        sizeof( uint8_t        ) );  		
		  	
		  	if( count_temp++ == 20000-1 )
				count_temp = 0;;
		}
		mem_flag = 1;
	}
	else{
    	for (i=0;i<8;i++)
  			memcpy( linearBuffer_dataToSocket2 + i, &sync_d, sizeof( sync_d ) );
  			// 今までの2000サンプル分のメモリ格納処理

		for(i=1;i<=TRANS_NUM;i++){
  			temp = count_temp;
  
  			data_sampleNo_MSByte = (uint8_t)( ( temp >> 8 ) & 0x00FF );
		  	data_sampleNo_LSByte = (uint8_t)( ( temp >> 0 ) & 0x00FF );
  
			memcpy( linearBuffer_dataToSocket2 + i*8+0, &data_sampleNo_LSByte, sizeof( data_sampleNo_LSByte ) );
	  		memcpy( linearBuffer_dataToSocket2 + i*8+1, &data_sampleNo_MSByte, sizeof( data_sampleNo_MSByte ) );
			memcpy( linearBuffer_dataToSocket2 + i*8+2, adr+(i-1)*6+0,        sizeof( uint8_t        ) );
  			memcpy( linearBuffer_dataToSocket2 + i*8+3, adr+(i-1)*6+1,        sizeof( uint8_t        ) );
  			memcpy( linearBuffer_dataToSocket2 + i*8+4, adr+(i-1)*6+2,        sizeof( uint8_t        ) );
		  	memcpy( linearBuffer_dataToSocket2 + i*8+5, adr+(i-1)*6+3,        sizeof( uint8_t        ) );
			memcpy( linearBuffer_dataToSocket2 + i*8+6, adr+(i-1)*6+4,        sizeof( uint8_t        ) );
	  		memcpy( linearBuffer_dataToSocket2 + i*8+7, adr+(i-1)*6+5,        sizeof( uint8_t        ) );  		
		  	
		  	if( count_temp++ == 20000-1 )
				count_temp = 0;;
		}
		mem_flag = 0;
	}
	
	#if 0
	else{
	// 今までの2000サンプル分のメモリ格納処理
	//for(i=0;i<TRANS_NUM;i++){
		for(i=1;i<=TRANS_NUM;i++){

  			temp = count_temp;
  
		  	data_sampleNo_MSByte = (uint8_t)( ( temp >> 8 ) & 0x00FF );
  			data_sampleNo_LSByte = (uint8_t)( ( temp >> 0 ) & 0x00FF );
  
		  	memcpy( linearBuffer_dataToSocket + i*8+0, &data_sampleNo_LSByte, sizeof( data_sampleNo_LSByte ) );
  			memcpy( linearBuffer_dataToSocket + i*8+1, &data_sampleNo_MSByte, sizeof( data_sampleNo_MSByte ) );
	  		memcpy( linearBuffer_dataToSocket + i*8+2, adr+(i-1)*6+0,        sizeof( uint8_t        ) );
	  		memcpy( linearBuffer_dataToSocket + i*8+3, adr+(i-1)*6+1,        sizeof( uint8_t        ) );
		  	memcpy( linearBuffer_dataToSocket + i*8+4, adr+(i-1)*6+2,        sizeof( uint8_t        ) );
  			memcpy( linearBuffer_dataToSocket + i*8+5, adr+(i-1)*6+3,        sizeof( uint8_t        ) );
		  	memcpy( linearBuffer_dataToSocket + i*8+6, adr+(i-1)*6+4,        sizeof( uint8_t        ) );
  			memcpy( linearBuffer_dataToSocket + i*8+7, adr+(i-1)*6+5,        sizeof( uint8_t        ) );  	
  	
		  	if( count_temp++ == 20000-1 )
  				count_temp = 0;;
  	
  		}
  	}
  	if( head_include++ == 9 )
  		head_include = 0;
  	#endif
  	//memcpy( linearBuffer_dataToSocket + TRANS_NUM*8, cr_data,        sizeof( uint8_t        ) );  	// VBへデータを渡す為に追加//やっぱり止め
  	
  	//printf("*adr =%2x %2x %2x %2x %2x %2x %2x %2x %2x %2x\n",*adr,*(adr+1),*(adr+2),*(adr+3),*(adr+4),*(adr+5),*(adr+6),*(adr+7),*(adr+8),*(adr+9) );  // comment out

  //amount_temp++;  //サンプル数を蓄積
}


uint8_t core0_flg=0;

// ********** Socketへデータ送信 **********
int transferData_ToSocket( void )
{
  //ESP_LOGI( logTag_transferData_ToSocket, "ringPointer_nextInputStartIndex  = %d", ringPointer_nextInputStartIndex );
  //ESP_LOGI( logTag_transferData_ToSocket, "ringPointer_nextOutputStartIndex = %d", ringPointer_nextOutputStartIndex );
  
  
  // データ送信
  //length_sentData = send( socket_connect_clientSide, linearBuffer_dataToSocket, length_sendData, 0 );
  //length_sentData = send( socket_connect_clientSide, linearBuffer_dataToSocket, 12000, 0 );
  //length_sentData = send( socket_connect_clientSide, linearBuffer_dataToSocket, TRANS_NUM*6, 0 );
  while(1){
  	if (core0_flg){
		#if 0
  		if (head_include == 0)
  			length_sentData = send( socket_connect_clientSide, linearBuffer_dataToSocket, (TRANS_NUM+1)*8, 0 );  // 2001 x 8byte wifi 転送する。
  		else 
  			length_sentData = send( socket_connect_clientSide, linearBuffer_dataToSocket, TRANS_NUM*8, 0 );
  		#endif
  		
		start = clock();
		starti = (int)start;
  		printf("Wifi Transfer start time to server = %d\n",starti);            // for monitor 2020.06.25
  		
  		//length_sentData = send( socket_connect_clientSide, linearBuffer_dataToSocket, (TRANS_NUM+1)*8, 0 );  // 2001 x 8byte wifi 転送する。
  		if(mem_flag==1){
  			length_sentData = send( socket_connect_clientSide, linearBuffer_dataToSocket, (TRANS_NUM+1)*8, 1 );  // 2001 x 8byte wifi 転送する。 // 2020.06.25

		// espで送る際、時間が掛かる時があるのを、発見。頻回している。0.1秒毎の転送が出来ていない。そのため、転送したデータを確認する。2020.06.25
			printf("%2x %2x %2x %2x %2x %2x %2x %2x ", linearBuffer_dataToSocket[0], linearBuffer_dataToSocket[1], linearBuffer_dataToSocket[2], linearBuffer_dataToSocket[3], linearBuffer_dataToSocket[4],linearBuffer_dataToSocket[5], linearBuffer_dataToSocket[6], linearBuffer_dataToSocket[7]);
			printf("%2x %2x %2x %2x %2x %2x %2x %2x ", linearBuffer_dataToSocket[8], linearBuffer_dataToSocket[9], linearBuffer_dataToSocket[10], linearBuffer_dataToSocket[11], linearBuffer_dataToSocket[12],linearBuffer_dataToSocket[13], linearBuffer_dataToSocket[14], linearBuffer_dataToSocket[15]);		
			printf("%2x %2x %2x %2x %2x %2x %2x %2x \n", linearBuffer_dataToSocket[16], linearBuffer_dataToSocket[17], linearBuffer_dataToSocket[18], linearBuffer_dataToSocket[19], linearBuffer_dataToSocket[20],linearBuffer_dataToSocket[21], linearBuffer_dataToSocket[22], linearBuffer_dataToSocket[23]);	
			end = clock();
			endi = (int)end;
			printf("Wifi Transfer End time to server = %d\n",endi);  // // for monitor 2020.06.25
		}
  		else{
  			length_sentData = send( socket_connect_clientSide, linearBuffer_dataToSocket2, (TRANS_NUM+1)*8, 1 );  // 2001 x 8byte wifi 転送する。 // 2020.06.25

		// espで送る際、時間が掛かる時があるのを、発見。頻回している。0.1秒毎の転送が出来ていない。そのため、転送したデータを確認する。2020.06.25
			printf("%2x %2x %2x %2x %2x %2x %2x %2x ", linearBuffer_dataToSocket2[0], linearBuffer_dataToSocket2[1], linearBuffer_dataToSocket2[2], linearBuffer_dataToSocket2[3], linearBuffer_dataToSocket2[4],linearBuffer_dataToSocket2[5], linearBuffer_dataToSocket2[6], linearBuffer_dataToSocket2[7]);
			printf("%2x %2x %2x %2x %2x %2x %2x %2x ", linearBuffer_dataToSocket2[8], linearBuffer_dataToSocket2[9], linearBuffer_dataToSocket2[10], linearBuffer_dataToSocket2[11], linearBuffer_dataToSocket2[12],linearBuffer_dataToSocket2[13], linearBuffer_dataToSocket2[14], linearBuffer_dataToSocket2[15]);		
			printf("%2x %2x %2x %2x %2x %2x %2x %2x \n", linearBuffer_dataToSocket2[16], linearBuffer_dataToSocket2[17], linearBuffer_dataToSocket2[18], linearBuffer_dataToSocket2[19], linearBuffer_dataToSocket2[20],linearBuffer_dataToSocket2[21], linearBuffer_dataToSocket2[22], linearBuffer_dataToSocket2[23]);	
			end = clock();
			endi = (int)end;
			printf("Wifi Transfer End time to server = %d\n",endi);  // // for monitor 2020.06.25
		}
  		
  		// 正常にサーバにデータを送信した
  		if( length_sentData > 0 )
  		{
    		ESP_LOGI( logTag_transferData_ToSocket, "PSH: %doctet", length_sentData );
    		memset( linearBuffer_dataToSocket, 0x00, sizeof( linearBuffer_dataToSocket ) );
    		ringPointer_nextOutputStartIndex = ringPointer_nextInputStartIndex;
    		core0_flg = 0;
  		}
  
  		// 送信すべきデータがない
  		else if( length_sentData == 0 )
  		{
    		ESP_LOGI( logTag_transferData_ToSocket, "FIN" );
  		}
  
  		// その他エラー
  		else
  		{
    		ESP_LOGI( logTag_transferData_ToSocket, "RST" );
  		}
  	}
  
  }
  
  return( length_sentData );
}

// ********** Socketへデータ送信 **********
int RecieveData_FromMaster( void )
{

        while (1) {
            int len = recv(socket_connect_clientSide, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // Error occured during receiving
            if (len < 0) {
                //ESP_LOGI(logTag_transferData_ToSocket, "recv failed: errno %d", errno);
                //ESP_LOGI(logTag_transferData_ToSocket, "recv failed: errno ");  // 2020.07.15 comment out
                break;
            }
            // Connection closed
            else if (len == 0) {
                //ESP_LOGI(logTag_transferData_ToSocket, "Connection closed");  // 2020.07.15 comment out
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                #if 0
                if (sourceAddr.sin6_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&sourceAddr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                } else if (sourceAddr.sin6_family == PF_INET6) {
                    inet6_ntoa_r(sourceAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
                }
                #endif 
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                //ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                //ESP_LOGI(logTag_transferData_ToSocket, "Received %d bytes", len);   // 2020.07.15 comment out
                //ESP_LOGI(logTag_transferData_ToSocket, "Received character %s", rx_buffer);  // 2020.07.15 comment out

                //int err = send(socket_connect_clientSide, rx_buffer, len, 0);
                int err = send(socket_connect_clientSide, send_chk_to_master, 8, 0);
		        //length_sentData = send( socket_connect_clientSide, linearBuffer_dataToSocket2, (TRANS_NUM+1)*8, 1 );  // 2001 x 8byte wifi 転送する。 // 2020.06.25
				if (err == 8)
					//ESP_LOGI(logTag_transferData_ToSocket, "Call Back OK");  // 2020.07.15 comment out
                if (err < 0) {
                    //ESP_LOGI(logTag_transferData_ToSocket, "Error occured during sending: errno %d", errno);  // 2020.07.15 comment out
                    break;
                }
            }
        }

		return 0;
}

static esp_err_t event_handler( void *ctx, system_event_t *event )
{
  switch( event->event_id )
  {
    case SYSTEM_EVENT_STA_START:
      // esp_wifi_start() が成功したときに呼ばれる。
      ESP_LOGI( logTag_ESP32_Event, "SYSTEM_EVENT_STA_START" );
      ESP_ERROR_CHECK( esp_wifi_connect() );
      break;
    
    case SYSTEM_EVENT_STA_CONNECTED:
      // esp_wifi_connect() が成功したときに呼ばれる。
      ESP_LOGI( logTag_ESP32_Event, "SYSTEM_EVENT_STA_CONNECTED" );
      // Event Taskは、自動的にDHCPv4クライアントを開始する。
      // IPv6 LLAをIFに付与する。
      tcpip_adapter_create_ip6_linklocal( TCPIP_ADAPTER_IF_STA );
      break;
    
    case SYSTEM_EVENT_STA_GOT_IP:
      // IPv4アドレスが正常に設定されたときに呼ばれる。
      ip4addr_ntoa_r( &event->event_info.got_ip.ip_info.ip, string_IPv4addr_DHCP, sizeof( string_IPv4addr_DHCP ) );
      ESP_LOGI( logTag_ESP32_Event, "SYSTEM_EVENT_STA_GOT_IP: %s", string_IPv4addr_DHCP );
      
      // IPv4 GUAが設定されたらアプリケーションを開始する。
      
      for_ipv4=1;   // flag set

      //init_TCP_Client();

      
      #if 0
      //if( string_IPv6addr_SLAAC[ 0 ] == '2' )
      if( 1 )
      {
        //init_systemTime();    // 2020.02.26
        init_TCP_Client();
        
        while( 1 )
        {
		  // time measure
  		  start = clock();
		  starti = (int)start;
  		  //printf("Start time = %lu\n",start);
  		  printf("Start time = %d\n",starti);
  
  		  getData_count = 0;
          //for( int count_loop = 0; count_loop < 6; count_loop++ )
          //for( int count_loop = 0; count_loop < 12; count_loop++ )
          //for( int count_loop = 0; count_loop < 24; count_loop++ )
          //for( int count_loop = 0; count_loop < 48; count_loop++ )
          //for( int count_loop = 0; count_loop < 96; count_loop++ )
          //for( int count_loop = 0; count_loop < 128; count_loop++ )
		  //for( int count_loop = 0; count_loop < 1024; count_loop++ )
		  //for( int count_loop = 0; count_loop < 2048; count_loop++)
		  for( int count_loop = 0; count_loop < 4096; count_loop++)     // OK max
		  //for( int count_loop = 0; count_loop < 8192; count_loop++ )  //NG
          {
            getData_fromKX224();
            getData_count++;
            //vTaskDelay( 1000 / portTICK_PERIOD_MS );   // 2020.02.28
          }
          
          // time measure
  		  end = clock();
  		  endi = (int)end;
  		  //printf("End time = %lu\n",end);
  		  printf("End time = %d\n",endi);
          
          // transfer time measure
  		  start = clock();
		  starti = (int)start;
  		  printf("Transfer start time to server = %d\n",starti);
          
          transferData_ToSocket();
          
          // Transfer time end measure
  		  end = clock();
  		  endi = (int)end;
  		  printf("Transfer End time to server = %d\n",endi);
          
          //vTaskDelay( 1000 / portTICK_PERIOD_MS );   // 2020.02.28
          vTaskDelay( 4000 / portTICK_PERIOD_MS );   // 2020.03.01
        }
        close( socket_connect_clientSide );
      }
      #endif

      break;
    
#if 0
    case SYSTEM_EVENT_GOT_IP6:
      // IPv6アドレスが正常に設定されたときに呼ばれる。
      // 1回目: Link-Local Address(fe80::/64)
      // 2回目: Global Unicast Address(2___:____:____:____::/64)
      ip6addr_ntoa_r( &event->event_info.got_ip6.ip6_info.ip, string_IPv6addr_SLAAC, sizeof( string_IPv6addr_SLAAC ) );
      ESP_LOGI( logTag_ESP32_Event, "SYSTEM_EVENT_GOT_IP6: %s", string_IPv6addr_SLAAC );

#if 0
      // IPv6 GUAが設定されたらアプリケーションを開始する。
      if( string_IPv6addr_SLAAC[ 0 ] == '2' )
      {
        //init_systemTime();    // 2020.02.26
        init_TCP_Client();
        
        while( 1 )
        {
		  // time measure
  		  start = clock();
		  starti = (int)start;
  		  //printf("Start time = %lu\n",start);
  		  printf("Start time = %d\n",starti);
  
  		  getData_count = 0;
          //for( int count_loop = 0; count_loop < 6; count_loop++ )
          //for( int count_loop = 0; count_loop < 12; count_loop++ )
          //for( int count_loop = 0; count_loop < 24; count_loop++ )
          //for( int count_loop = 0; count_loop < 48; count_loop++ )
          //for( int count_loop = 0; count_loop < 96; count_loop++ )
          //for( int count_loop = 0; count_loop < 128; count_loop++ )
		  //for( int count_loop = 0; count_loop < 1024; count_loop++ )
		  //for( int count_loop = 0; count_loop < 2048; count_loop++)
		  for( int count_loop = 0; count_loop < 4096; count_loop++)     // OK max
		  //for( int count_loop = 0; count_loop < 8192; count_loop++ )  //NG
          {
            getData_fromKX224();
            getData_count++;
            //vTaskDelay( 1000 / portTICK_PERIOD_MS );   // 2020.02.28
          }
          
          // time measure
  		  end = clock();
  		  endi = (int)end;
  		  //printf("End time = %lu\n",end);
  		  printf("End time = %d\n",endi);
          
          // transfer time measure
  		  start = clock();
		  starti = (int)start;
  		  printf("Transfer start time to server = %d\n",starti);
          
          transferData_ToSocket();
          
          // Transfer time end measure
  		  end = clock();
  		  endi = (int)end;
  		  printf("Transfer End time to server = %d\n",endi);
          
          //vTaskDelay( 1000 / portTICK_PERIOD_MS );   // 2020.02.28
          vTaskDelay( 2000 / portTICK_PERIOD_MS );   // 2020.03.01
        }
        close( socket_connect_clientSide );
      }
#endif
      break;
#endif

    case SYSTEM_EVENT_STA_DISCONNECTED:
      // Wi-Fiが切断されたときに呼ばれる。
      ESP_LOGI( logTag_ESP32_Event, "SYSTEM_EVENT_STA_DISCONNECTED" );
      // Event Taskは、全てのSocketでErrorを返す。
      ESP_ERROR_CHECK( esp_wifi_connect() );
      break;
    
    default:
      break;
  }
  
  return ESP_OK;
}




//Called after a transaction is queued and ready for pickup by master. We use this to set the handshake line high.
void my_post_setup_cb(spi_slave_transaction_t *trans) {
    WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1<<GPIO_HANDSHAKE));
}

//Called after transaction is sent/received. We use this to set the handshake line low.
void my_post_trans_cb(spi_slave_transaction_t *trans) {
    WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1<<GPIO_HANDSHAKE));
}


#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
//#define TIMER_INTERVAL0_SEC   (3.4179) // sample test interval for the first timer
//#define TIMER_INTERVAL0_SEC   (0.00005) // sample test interval for the first timer
#define TIMER_INTERVAL0_SEC   (0.1) // sample test interval for the first timer
#define TIMER_INTERVAL1_SEC   (5.78)   // sample test interval for the second timer
#define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

#define ADC1_TEST_CHANNEL (4) // GPIO32=ADC1のチャンネル4を使用

int a=0;
int timetemp=0;
/*
 * A sample structure to pass events
 * from the timer interrupt handler to the main program.
 */
typedef struct {
    int type;  // the type of timer's event
    int timer_group;
    int timer_idx;
    uint64_t timer_counter_value;
} timer_event_t;

xQueueHandle timer_queue;

/*
 * A simple helper function to print the raw timer counter value
 * and the counter value converted to seconds
 */
static void inline print_timer_counter(uint64_t counter_value)
{
    printf("Counter: 0x%08x%08x\n", (uint32_t) (counter_value >> 32),
                                    (uint32_t) (counter_value));
    printf("Time   : %.8f s\n", (double) counter_value / TIMER_SCALE);
}

/*
 * Timer group0 ISR handler
 *
 * Note:
 * We don't call the timer API here because they are not declared with IRAM_ATTR.
 * If we're okay with the timer irq not being serviced while SPI flash cache is disabled,
 * we can allocate this interrupt without the ESP_INTR_FLAG_IRAM flag and use the normal API.
 */

/*
 * Initialize selected timer of the timer group 0
 *
 * timer_idx - the timer number to initialize
 * auto_reload - should the timer auto reload on alarm?
 * timer_interval_sec - the interval of alarm to set
 */


uint8_t data_t[TRANS_NUM*6]; // for test of transfer

/*
 * The main task of this example program
 */


uint8_t count10=0;


void kx224_wifi(void)
{
    while (1) {
        timer_event_t evt;

        spi_slave_transaction_t t;
     	esp_err_t ret;
     	
     	uint8_t* sendbuf;
	    uint8_t* recvbuf;
	    
	    uint16_t chkdata;
	    uint8_t upd,dwn;
      
        //xQueueReceive(timer_queue, &evt, portMAX_DELAY);  // これを外さないと、errorになる。注意。
        
		//sendbuf = (uint8_t*)heap_caps_malloc(12000,MALLOC_CAP_DMA);
		//recvbuf = (uint8_t*)heap_caps_malloc(12000,MALLOC_CAP_DMA);
		sendbuf = (uint8_t*)heap_caps_malloc(TRANS_NUM*6,MALLOC_CAP_DMA);
		recvbuf = (uint8_t*)heap_caps_malloc(TRANS_NUM*6,MALLOC_CAP_DMA);
        //memset(recvbuf, 0xA5, 129);

		#if 1  // 2020.05.05

        #if 1
        //t.length=12000*8;
        t.length=(TRANS_NUM*6)*8;  // 2020.05.16変更 picの転送バイト数80byteに合わせた。
        //t.length=(TRANS_NUM*8)*8;
        #endif
        t.tx_buffer=sendbuf;
        t.rx_buffer=recvbuf;

        ret=spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);

		#if 0  // 2020.06.25
        printf("%2x %2x %2x %2x %2x %2x ", recvbuf[0], recvbuf[1], recvbuf[2], recvbuf[3], recvbuf[4], recvbuf[5]); // comment out
        //printf("%2x %2x %2x %2x %2x %2x ", recvbuf[6], recvbuf[7], recvbuf[8], recvbuf[9], recvbuf[10], recvbuf[11]); // comment out
        printf("%2x %2x %2x %2x %2x %2x ", recvbuf[12], recvbuf[13], recvbuf[14], recvbuf[15], recvbuf[16], recvbuf[17]); // comment out
        //printf("%2x %2x %2x %2x %2x %2x \n", recvbuf[18], recvbuf[19], recvbuf[20], recvbuf[21], recvbuf[22], recvbuf[23]); // comment out
		printf("%2x %2x %2x %2x %2x %2x \n", recvbuf[24], recvbuf[25], recvbuf[26], recvbuf[27], recvbuf[28], recvbuf[29]); // comment out
		#endif  // 2020.06.25

		#endif

        #if 1
        
        #if 0  // 三角波20000サンプルの転送 2020.05.08
        int i;
        for (i=0;i<TRANS_NUM;i++){
        	chkdata=(count10 * 2000 + i)*3;
        	upd = chkdata / 256;
	        dwn = chkdata % 256;
    	    data_t[i*6+0]=dwn;
        	data_t[i*6+1]=upd;
	        data_t[i*6+2]=dwn;
    	    data_t[i*6+3]=upd;
        	data_t[i*6+4]=dwn;
	        data_t[i*6+5]=upd;
        }
        getData_fromKX224_2(&data_t);
        
        if(count10++==9)
        	count10 = 0;
        
        #endif

        #if 0  // 2020.05.16 これを送る。
        int i;
        for (i=0;i<TRANS_NUM*6;i++)
        data_t[i]=48 + i % 6;
        getData_fromKX224_2(&data_t);
        #endif
        
        #if 1  // 2020.05.16 人工データを送るので、コメントアウト。
        getData_fromKX224_2(recvbuf);
        #endif

        
        //memcpy( linearBuffer_dataToSocket,  recvbuf, 12000 );
        
        // transfer time measure
        #if 0   // 2020.06.25
  		start = clock();
		starti = (int)start;
  		printf("Transfer start time to server = %d\n",starti);            // for monitor 2020.04.16
        #endif
        
        // for dual core
        //transferData_ToSocket();  // 2020.05.03
        core0_flg = 1;
        ///////////////////////////////////////////
        
        // Transfer time end measure
        #if 0   // 2020.06.25
	    end = clock();
	    endi = (int)end;
	    printf("Transfer End time to server = %d\n",endi);  // for monitor 2020.04.16
	    #endif
	    
        #endif
        
        printf("access OK \n");

        free(sendbuf);
        free(recvbuf);

    }
}


void app_main( void )
{
	uint8_t data_Who_am_i = 0;
  	//unsigned char rc;
 	unsigned char reg=0;
  	unsigned char gsel=0;

    // for gpio start 2020.03.20
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    //gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);  //2020.04.01


//	clock_t start,end;

  // (initStep. 1) I2C Master Init Phase
  
  //ESP_ERROR_CHECK( i2c_param_config( I2C_NUM_0, &i2c_config ) );
  //ESP_ERROR_CHECK( i2c_driver_install( I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0 ) );
  // 引数1: (i2c_port_t)i2c_num   : I2C_NUM_0       // I2Cポート番号
  // 引数2: (i2c_mode_t)mode      : I2C_MODE_MASTER // I2C動作モード
  // 引数3: (size_t)slv_rx_buf_len: 0               // Slave動作時の受信バッファ（Masterモードなので不要）
  // 引数4: (size_t)slv_tx_buf_len: 0               // Slave動作時の送信バッファ（Masterモードなので不要）
  // 引数3: (int)intr_alloc_flags : 0               // ESP32の割り込みフラグ（使わない）
  
  
  
  // (initStep. 2) I2C Slave (INA226) Init Phase
  
  // INA226 Configuration Registerを設定
  // 平均化点数: 1024点
  // V変換時間: 8.244ms
  // I変換時間: 8.244ms
  // 測定モード: IVともに連続測定
  // $ i2cset -y 1 0x40 0x00 0x4F 0xFF i
  
  
  // time measure
  //start = clock();
  //printf("Start time = %lu\n",start);
  
   
  
  #if 1
  // (initStep. 3) Wi-Fi/LwIP Init Phase
  
  tcpip_adapter_init();
  ESP_ERROR_CHECK( esp_event_loop_init( event_handler, NULL ) );
  
  // Wi-Fi PHYを初期化する
  // NVS（不揮発ストレージ）を使わない。
  wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
  wifi_init_cfg.nvs_enable = false;
  ESP_ERROR_CHECK( esp_wifi_init( &wifi_init_cfg ) );
  
  
  
  // (initStep. 4) Wi-Fi Configuration Phase
  
  // Wi-FiのCountry Codeを設定する
  // Country Codeは esp_wifi_init() の後でなければならない。
  ESP_ERROR_CHECK( esp_wifi_set_country( &wifi_country ) );
  
  // Wi-Fi MACを設定する
  ESP_ERROR_CHECK( esp_wifi_set_mode( WIFI_MODE_STA ) );
  ESP_ERROR_CHECK( esp_wifi_set_config( ESP_IF_WIFI_STA, &wifi_config ) );
  
  
  
  // (initStep. 5) Wi-Fi Start Phase
  
  ESP_ERROR_CHECK( esp_wifi_start() );
  #endif
  
  
  
      esp_err_t ret;

    //Configuration for the SPI bus
    spi_bus_config_t buscfg={
        .mosi_io_num=GPIO_MOSI,
        .miso_io_num=GPIO_MISO,
        .sclk_io_num=GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 16384
    };

    //Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg={
        .mode=0,
        .spics_io_num=GPIO_CS,
        .queue_size=3,
        .flags=0,
        .post_setup_cb=my_post_setup_cb,
        .post_trans_cb=my_post_trans_cb
        
    };

    //Configuration for the handshake line
    gpio_config_t io_conf_a={
        .intr_type=GPIO_INTR_DISABLE,
        .mode=GPIO_MODE_OUTPUT,
        .pin_bit_mask=(1<<GPIO_HANDSHAKE)
    };

    //Configure handshake line as output
    gpio_config(&io_conf_a);
    //Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    //Initialize SPI slave interface
    ret=spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, DMA_CHAN);
    //ret=spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, 0);
    assert(ret==ESP_OK);



    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));
  
  
  int run1st=0;
  
  int ii=0;
  
  for (ii=0;ii<8;ii++)
	send_chk_to_master[ii] = 0xFE;
  //for test
  //gpio_set_level(GPIO_OUTPUT_IO_0, 1);
  gpio_set_level(GPIO_OUTPUT_IO_0, 0);
  
  while(1){

    if((for_ipv4 == 1) && (run1st == 0)){

		gpio_set_level(GPIO_OUTPUT_IO_1, 1);
		printf("this is first time\n");
		run1st = 1;
		init_TCP_Client();
		
		//xTaskCreatePinnedToCore(kx224_wifi,"Kx224_wifi",4096,NULL,1,NULL,1);
		xTaskCreatePinnedToCore(kx224_wifi,"Kx224_wifi",4096,NULL,10,NULL,0);
		xTaskCreatePinnedToCore(transferData_ToSocket,"TransferData_ToSocket",4096,NULL,1,NULL,0);
		//xTaskCreatePinnedToCore(transferData_ToSocket,"TransferData_ToSocket",8196,NULL,1,NULL,0);
  		//kx224_wifi();

		xTaskCreatePinnedToCore(RecieveData_FromMaster,"RecieveData_FromMaster",4096,NULL,1,NULL,0); // PC画面が乱れるので削除してみた 2020.07.05

		break;

	}

  }
  
 
}
