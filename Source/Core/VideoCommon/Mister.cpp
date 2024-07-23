#include "Mister.h"

MiSTer::MiSTer()
{
}

MiSTer::~MiSTer()
{
}

char* MiSTer::getPBufferBlit(void)
{
   return gmw_get_pBufferBlit();     
}

char* MiSTer::getPBufferAudio(void)
{
   return gmw_get_pBufferAudio();     	
}

void MiSTer::Close(void)
{   	 
   gmw_close();
}

void MiSTer::Init(const char* mister_host, short mister_port, uint8_t lz4_frames, uint32_t sound_rate, uint8_t sound_chan, uint8_t rgb_mode)
{   
   printf("rgb_mode %d\n",rgb_mode);
   gmw_init(mister_host, lz4_frames, sound_rate, sound_chan, rgb_mode, 0);
   gmw_set_log_level(0);

   is_init = true;
}

bool MiSTer::isInit() {
   return is_init;
}

void MiSTer::CmdSwitchres240p()
{
    // Hardcoded from switchres -c 640 240 60 -m ntsc
    // Switchres: Modeline "640x240_59 15.704280KHz 59.940000Hz" 12.123704 640 658 715 772 240 243 246 262   -hsync -vsync

    double px = 12.123704;
    uint16_t hactive = 640;
    uint16_t hbegin = 658;
    uint16_t hend = 715;
    uint16_t htotal = 775;
    uint16_t vactive = 240;
    uint16_t vbegin = 243;
    uint16_t vend = 246;
    uint16_t vtotal = 262;
    uint8_t  interlace = 0;

    gmw_switchres(px, hactive, hbegin, hend, htotal, vactive, vbegin, vend, vtotal, interlace);

    video_mode = V240p;
    frameField = 0;
}

void MiSTer::CmdSwitchres480i()
{
    // Hardcoded from switchres -c 640 480 60 -m ntsc
    // Switchres: Modeline "640x480_59i 15.734250KHz 59.940000Hz" 12.146841 640 658 715 772 480 487 493 525 interlace  -hsync -vsync

    double px = 12.146841;
    uint16_t hactive = 640;
    uint16_t hbegin = 658;
    uint16_t hend = 715;
    uint16_t htotal = 772;
    uint16_t vactive = 480;
    uint16_t vbegin = 487;
    uint16_t vend = 493;
    uint16_t vtotal = 525;
    uint8_t  interlace = 1;

    gmw_switchres(px, hactive, hbegin, hend, htotal, vactive, vbegin, vend, vtotal, interlace);

    video_mode = V480i;
    frameField = 0;
}

void MiSTer::CmdSwitchres480p()
{
    // VGA modeline.
    // ModeLine "640x480_60" 25.174825174825 640 656 752 800 480 490 492 525 -hsync -vsync

    double px = 25.174825174825;
    uint16_t hactive = 640;
    uint16_t hbegin = 656;
    uint16_t hend = 752;
    uint16_t htotal = 800;
    uint16_t vactive = 480;
    uint16_t vbegin = 490;
    uint16_t vend = 492;
    uint16_t vtotal = 525;
    uint8_t  interlace = 0; // 2?

    gmw_switchres(px, hactive, hbegin, hend, htotal, vactive, vbegin, vend, vtotal, interlace);
    
    video_mode = V480p;
    frameField = 0;
}


void MiSTer::Blit(uint16_t vsync)
{
   frame++;
   gmw_fpgaStatus status;
   gmw_getStatus(&status);
   if (status.frame > frame) frame = status.frame + 1;
   gmw_blit(frame, vsync, 0);
}

void MiSTer::Audio(uint32_t sizeSound, uint8_t soundchan)
{
   uint16_t bytesSound = sizeSound * soundchan * 2;
   gmw_audio(bytesSound);
}

void MiSTer::Sync()
{
   gmw_waitSync();
}

int MiSTer::GetField(void)
{
	int field = 0;
	if(video_mode == V480i) // interlaced
	{
		frameField = !frameField;
		field = frameField;
	}

	return field;
}

MiSTerVideoMode MiSTer::getVideoMode(void) {
   return video_mode;
}

MiSTer g_mister;
