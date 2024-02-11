#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>
#include <math.h>
#include <cstring> //memcpy
#include <array>
#include <mutex>

#define USE_OVERLAPPED 0

#ifdef WIN32
 #include <winsock2.h>
 #include <ws2tcpip.h>
#else
 #include <netinet/udp.h>
 #include <sys/types.h>
 #include <sys/socket.h>
 #include <arpa/inet.h>
 #include <netinet/in.h>
 #include <fcntl.h>

 #include <sys/time.h>
 #include <sys/stat.h>
 #include <time.h>
 #include <unistd.h>
#endif

#include <lz4.h>
#include <lz4hc.h>

#ifdef WIN32
 #define SR_WIN32_STATIC
#endif

#define CMD_CLOSE 1
#define CMD_INIT 2
#define CMD_SWITCHRES 3
#define CMD_AUDIO 4
#define CMD_GET_STATUS 5
#define CMD_BLIT_VSYNC 6


#define MAX_BUFFER_WIDTH 1024
#define MAX_BUFFER_HEIGHT 768
#define MAX_LZ4_BLOCK   61440

#define MISTER_SOUND_BUFFER_SIZE 32*4096

typedef union
{
  struct
  {
    unsigned char bit0 : 1;
    unsigned char bit1 : 1;
    unsigned char bit2 : 1;
    unsigned char bit3 : 1;
    unsigned char bit4 : 1;
    unsigned char bit5 : 1;
    unsigned char bit6 : 1;
    unsigned char bit7 : 1;
  }u;
   uint8_t byte;
} bitByte;

class MiSTer
{
 public:
 MiSTer();
 ~MiSTer();

 void CmdClose(void);
 void CmdInit(const char* mister_host, short mister_port, bool lz4_frames, uint32_t sound_rate, uint8_t sound_chan);
 void CmdSwitchres240p();
 void CmdSwitchres480i();
 void CmdBlit(char *bufferFrame, uint16_t vsync);
 void CmdAudio(const void *bufferFrame, uint32_t sizeSound, uint8_t soundchan);

 void SetStartEmulate(void); //TS before frame emulation
 void SetEndEmulate(void); //TS after frame emulation
 void SetStartBlit(void); //TS before process to send rgb (includes buffer + compress + udp)
 void SetEndBlit(void); //TS after process to send rgb (includes buffer + compress + udp)

 void Sync(void); //Sync emulator with MiSTer raster
 int  GetVSyncDif(void); //raster dif. between emulator and MiSTer (usec)

 int GetField(void);
 bool isInterlaced(void);
 bool is480p(void);
 bool isDownscaled(void);

 std::mutex m_audio_buffer_mutex;
 std::array<short, MISTER_SOUND_BUFFER_SIZE> m_audio_buffer;
 size_t audio_buffer_len;

 private:

 bool lz4_compress = false;
 uint32_t frame = 0;
 uint8_t  frameField = 0;
 uint16_t width = 0;
 uint16_t height = 0;
 uint16_t lines = 0; //vtotal
 uint8_t  interlaced = 0;
 uint8_t  downscaled = 0;
 uint32_t widthTime = 0; //usec
 uint32_t frameTime = 0; //usec

 uint32_t emulationTime = 0;
 uint32_t avgEmulationTime = 0;
 uint32_t blitTime = 0;

 uint16_t vsync_auto = 120;

 //ACK Status
 uint32_t frameEcho = 0;
 uint16_t vcountEcho = 0;
 uint32_t frameGPU = 0;
 uint16_t vcountGPU = 0;

 bool firstField = false;

 //FPGA debug bits
 uint8_t fpga_debug_bits = 0;
 uint8_t fpga_vram_end_frame = 0;
 uint8_t fpga_vram_ready = 0;
 uint8_t fpga_vram_synced = 0;
 uint8_t fpga_vga_frameskip = 0;
 uint8_t fpga_vga_vblank = 0;
 uint8_t fpga_vga_f1 = 0;
 uint8_t fpga_audio = 0;
 uint8_t fpga_vram_queue = 0;

 //UDP
 char bufferRecv[13];
 struct sockaddr_in ServerAddr;
 int sockfd;

 #ifdef WIN32
	 WSABUF DataBuf;
	 WSABUF DataBufRecv;
	 WSAOVERLAPPED Overlapped;
	 WSAOVERLAPPED OverlappedRecv;
	 SOCKET sockfdOV;
	 DWORD BytesRecv;
	 int ACKComplete;
	 int sizeof_ServerAddr;
 #endif

 //Time
 #ifdef WIN32
	 LARGE_INTEGER tickStartEmulate;
         LARGE_INTEGER tickEndEmulate;
         LARGE_INTEGER tickLastSync;
         LARGE_INTEGER tickStartBlit;
         LARGE_INTEGER tickEndBlit;
 #else
         struct timespec tickStartEmulate;
         struct timespec tickEndEmulate;
         struct timespec tickLastSync;
         struct timespec tickStartBlit;
         struct timespec tickEndBlit;
 #endif


 //LZ4
 char m_fb[MAX_BUFFER_HEIGHT * MAX_BUFFER_WIDTH * 3];
 char m_fb_compressed[MAX_BUFFER_HEIGHT * MAX_BUFFER_WIDTH * 3];
 //char inp_buf[2][MAX_BUFFER_WIDTH * 16 * 3 + 1];
 char inp_buf[2][MAX_LZ4_BLOCK + 1];


 //Internal functions
 //UDP
 void Send(void *cmd, int cmdSize);
 void SendMTU(char *buffer, int bytes_to_send, int chunk_max_size);
 void SendLZ4(char *buffer, int bytes_to_send, int block_size);
 void ReceiveBlitACK(void);

 //Time
 #ifndef WIN32
  uint32_t DiffTimespec(timespec start, timespec end); //nanosec
 #endif

};

extern MiSTer g_mister;
