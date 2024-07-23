#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>
#include <math.h>
#include <memory.h>
#include <array>
#include <mutex>

#include "groovymister_wrapper.h"

// From groovy api BUFFER_SIZE 1245312, used
// for both audio and video.
#define MISTER_SOUND_BUFFER_SIZE 1245312
#define MISTER_VIDEO_BUFFER_SIZE 1245312

enum MiSTerVideoMode {V240p, V480i, V480p};

class MiSTer
{
public:
  MiSTer();
  ~MiSTer();

  char* getPBufferBlit(void);    
  char* getPBufferAudio(void);

  void Close();
  void Init(const char* mister_host, short mister_port, uint8_t lz4_frames,
            uint32_t sound_rate, uint8_t sound_chan, uint8_t rgb_mode);

  void CmdSwitchres240p();
  void CmdSwitchres480i();
  void CmdSwitchres480p();

  void Blit(uint16_t vsync);
  void Audio(uint32_t sizeSound, uint8_t soundchan);
  void Sync(void);
  bool isInit();
  int GetField(void);
  MiSTerVideoMode getVideoMode(void);

  std::mutex m_audio_buffer_mutex;
  size_t audio_buffer_len;

private:

  bool is_init = false;
  MiSTerVideoMode video_mode = V240p;
  uint32_t frame = 0;
  uint8_t frameField = 0;
};

extern MiSTer g_mister;
