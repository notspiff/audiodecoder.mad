/*
 *      Copyright (C) 2005-2013 Team XBMC
 *      http://xbmc.org
 *
 *  This Program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *
 *  This Program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with XBMC; see the file COPYING.  If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 */

#include "kodi/libXBMC_addon.h"

#include "kodi/kodi_audiodec_dll.h"
#include "kodi/AEChannelData.h"
#include <mad.h>

ADDON::CHelper_libXBMC_addon *XBMC           = NULL;

extern "C" {

#include <math.h>
  
#define BYTES2INT(b1,b2,b3,b4) (((b1 & 0xFF) << (3*8)) | \
((b2 & 0xFF) << (2*8)) | \
((b3 & 0xFF) << (1*8)) | \
((b4 & 0xFF) << (0*8)))

#define UNSYNC(b1,b2,b3,b4) (((b1 & 0x7F) << (3*7)) | \
((b2 & 0x7F) << (2*7)) | \
((b3 & 0x7F) << (1*7)) | \
((b4 & 0x7F) << (0*7)))

#define MPEG_VERSION2_5 0
#define MPEG_VERSION1   1
#define MPEG_VERSION2   2

/* Xing header information */
#define VBR_FRAMES_FLAG 0x01
#define VBR_BYTES_FLAG  0x02
#define VBR_TOC_FLAG    0x04

// mp3 header flags
#define SYNC_MASK (0x7ff << 21)
#define VERSION_MASK (3 << 19)
#define LAYER_MASK (3 << 17)
#define PROTECTION_MASK (1 << 16)
#define BITRATE_MASK (0xf << 12)
#define SAMPLERATE_MASK (3 << 10)
#define PADDING_MASK (1 << 9)
#define PRIVATE_MASK (1 << 8)
#define CHANNELMODE_MASK (3 << 6)
#define MODE_EXT_MASK (3 << 4)
#define COPYRIGHT_MASK (1 << 3)
#define ORIGINAL_MASK (1 << 2)
#define EMPHASIS_MASK 3

#define DECODER_DELAY 529 // decoder delay in samples

#define DEFAULT_CHUNK_SIZE 16384

#define DECODING_ERROR    -1
#define DECODING_SUCCESS   0
#define DECODING_CALLAGAIN 1

#define SAMPLESPERFRAME   1152
#define CHANNELSPERSAMPLE 2
#define BITSPERSAMPLE     32
#define OUTPUTFRAMESIZE   (SAMPLESPERFRAME * CHANNELSPERSAMPLE * (BITSPERSAMPLE >> 3))

//-- Create -------------------------------------------------------------------
// Called on load. Addon should fully initalize or return error status
//-----------------------------------------------------------------------------
ADDON_STATUS ADDON_Create(void* hdl, void* props)
{
  if (!XBMC)
    XBMC = new ADDON::CHelper_libXBMC_addon;

  if (!XBMC->RegisterMe(hdl))
  {
    delete XBMC, XBMC=NULL;
    return ADDON_STATUS_PERMANENT_FAILURE;
  }

  return ADDON_STATUS_OK;
}

//-- Stop ---------------------------------------------------------------------
// This dll must cease all runtime activities
// !!! Add-on master function !!!
//-----------------------------------------------------------------------------
void ADDON_Stop()
{
}

//-- Destroy ------------------------------------------------------------------
// Do everything before unload of this add-on
// !!! Add-on master function !!!
//-----------------------------------------------------------------------------
void ADDON_Destroy()
{
}

//-- HasSettings --------------------------------------------------------------
// Returns true if this add-on use settings
// !!! Add-on master function !!!
//-----------------------------------------------------------------------------
bool ADDON_HasSettings()
{
  return false;
}

//-- GetStatus ---------------------------------------------------------------
// Returns the current Status of this visualisation
// !!! Add-on master function !!!
//-----------------------------------------------------------------------------
ADDON_STATUS ADDON_GetStatus()
{
  return ADDON_STATUS_OK;
}

//-- GetSettings --------------------------------------------------------------
// Return the settings for XBMC to display
// !!! Add-on master function !!!
//-----------------------------------------------------------------------------
unsigned int ADDON_GetSettings(ADDON_StructSetting ***sSet)
{
  return 0;
}

//-- FreeSettings --------------------------------------------------------------
// Free the settings struct passed from XBMC
// !!! Add-on master function !!!
//-----------------------------------------------------------------------------

void ADDON_FreeSettings()
{
}

//-- SetSetting ---------------------------------------------------------------
// Set a specific Setting value (called from XBMC)
// !!! Add-on master function !!!
//-----------------------------------------------------------------------------
ADDON_STATUS ADDON_SetSetting(const char *strSetting, const void* value)
{
  return ADDON_STATUS_OK;
}

//-- Announce -----------------------------------------------------------------
// Receive announcements from XBMC
// !!! Add-on master function !!!
//-----------------------------------------------------------------------------
void ADDON_Announce(const char *flag, const char *sender, const char *message, const void *data)
{
}

enum madx_sig {
  ERROR_OCCURED,
  MORE_INPUT,
  FLUSH_BUFFER,
  CALL_AGAIN,
  SKIP_FRAME
};

struct madx_house {
  struct mad_stream stream;
  struct mad_frame  frame;
  struct mad_synth  synth;
  mad_timer_t       timer;
  unsigned long     frame_cnt;
  unsigned char*    output_ptr;
};

struct madx_stat {
  size_t write_size;
  size_t readsize;
  size_t remaining;
  size_t framepcmsize;
  bool   flushed;
};

struct MADContext
{
  madx_house mxhouse;
  madx_stat mxstat;
  madx_sig mxsig;
  void* file;
  bool eof;
  bool Decoding;
  bool CallAgainWithSameBuffer;
  int readRetries;
  unsigned int FormatData[8];
  unsigned char flushcnt;
  bool HaveData;
  int64_t bufferpos;
  int iFirstSample;
  int iLastSample;
  std::vector<float> SeekOffset;
  float fTotalDuration;

  uint8_t InputBuffer[65536];
  size_t InputBufferPos;

  uint8_t OutputBuffer[4*OUTPUTFRAMESIZE];
  size_t OutputBufferPos;

  // Gapless playback
  bool IgnoreFirst;     // Ignore first samples if this is true (for gapless playback)
  bool IgnoreLast;      // Ignore first samples if this is true (for gapless playback)
  int IgnoredBytes;     // amount of samples ignored thus far

  MADContext()
  {
    file = NULL;
    flushcnt = 0;
    eof = false;
    HaveData = false;
    iFirstSample = iLastSample = 0;
    IgnoredBytes = 0;
    IgnoreFirst = IgnoreLast = false;
    readRetries = 0;
    Decoding = CallAgainWithSameBuffer = false;
    fTotalDuration = 0;
    InputBufferPos = 0;
    OutputBufferPos = 0;
    memset(&mxhouse, 0, sizeof(madx_house));
    memset(&mxstat, 0, sizeof(madx_stat));
  }

  void SetOffsets(int iSeekOffsets, const float* offsets)
  {
    SeekOffset.resize(iSeekOffsets+1);
    std::copy(offsets, offsets+iSeekOffsets+1, SeekOffset.begin());
  }

  int64_t GetByteOffset(MADContext* ctx, float fTime)
  {
    if (SeekOffset.empty())
      return 0;
    if (fTime > fTotalDuration)
      fTime = fTotalDuration;
    float fOffset = (fTime / fTotalDuration) * SeekOffset.size();
    int iOffset = (int)floor(fOffset);
    if (iOffset > SeekOffset.size()-1)
      iOffset = SeekOffset.size() - 1;
    float fa = SeekOffset[iOffset];
    float fb = SeekOffset[iOffset + 1];
    return (int64_t)(fa + (fb - fa) * (fOffset - iOffset));
  }

  int64_t GetTimeOffset(int64_t iBytes)
  {
    if (SeekOffset.empty())
      return 0;  // no seek info

    float fBytes = (float)iBytes;
    if (fBytes > SeekOffset.back())
      fBytes = SeekOffset.back();
    if (fBytes < SeekOffset[0])
      fBytes = SeekOffset[0];
    // run through our byte offsets searching for our times...
    int iOffset = 1;
    while (iOffset < SeekOffset.size() && fBytes > SeekOffset[iOffset])
      iOffset++;
    // iOffset will be the last of the two offsets and will be bigger than 1.
    float fTimeOffset = (float)iOffset - 1 + (fBytes - SeekOffset[iOffset - 1])/(SeekOffset[iOffset] - SeekOffset[iOffset - 1]);
    float fTime = fTimeOffset / SeekOffset.size() * fTotalDuration;
    return (int64_t)(fTime * 1000.0f);
  }

  void Flush()
  {
    mad_frame_mute(&mxhouse.frame);
    mad_synth_mute(&mxhouse.synth);
    mad_stream_finish(&mxhouse.stream);
    mad_stream_init(&mxhouse.stream);
    memset(&mxstat, 0, sizeof(madx_stat));
    mxstat.flushed = true;
    flushcnt = std::max(flushcnt+1, 2);
    HaveData = false;
    bufferpos = 0;
  }

  madx_sig madx_read(madx_house *mxhouse, madx_stat *mxstat, int maxwrite)
  {
    mxhouse->output_ptr = OutputBuffer + OutputBufferPos;

    if( mad_frame_decode(&mxhouse->frame, &mxhouse->stream) )
    {
      if( !MAD_RECOVERABLE(mxhouse->stream.error) )
      {
        if( mxhouse->stream.error == MAD_ERROR_BUFLEN )
        {    
          //printf("Need more input (%s)",  mad_stream_errorstr(&mxhouse->stream));
          mxstat->remaining = mxhouse->stream.bufend - mxhouse->stream.next_frame;

          return(MORE_INPUT);      
        }
        else
        {
          XBMC->Log(ADDON::LOG_ERROR, "(MAD)Unrecoverable frame level error (%s).", mad_stream_errorstr(&mxhouse->stream));
          return(ERROR_OCCURED); 
        }
      }
      return(SKIP_FRAME); 
    }

    mad_synth_frame( &mxhouse->synth, &mxhouse->frame );

    mxstat->framepcmsize = mxhouse->synth.pcm.length * mxhouse->synth.pcm.channels * (int)(BITSPERSAMPLE >> 3);
    mxhouse->frame_cnt++;
    mad_timer_add( &mxhouse->timer, mxhouse->frame.header.duration );

    float *dest = (float *)mxhouse->output_ptr;
    for(int i=0; i < mxhouse->synth.pcm.length; i++)
    {
      // Left channel
      *dest++ = (float)mad_f_todouble(mxhouse->synth.pcm.samples[0][i]);

      // Right channel
      if(MAD_NCHANNELS(&mxhouse->frame.header) == 2)
        *dest++ = (float)mad_f_todouble(mxhouse->synth.pcm.samples[1][i]);
    }

    // Tell calling code buffer size
    mxhouse->output_ptr = (unsigned char*)dest;
    mxstat->write_size  = mxhouse->output_ptr - (OutputBuffer + OutputBufferPos);

    return(FLUSH_BUFFER);
  }

  int Decode(int *out_len)
  {
    if (!HaveData)
    {
      //MAD needs padding at the end of the stream to decode the last frame, this doesn't hurt winamps in_mp3.dll
      int madguard = 0;
      if (eof)
      {
        madguard = 8;
        if (InputBufferPos + madguard > 65536)
          madguard = 65536 - InputBufferPos;
        memset(InputBuffer + InputBufferPos, 0, madguard);
      }

      mad_stream_buffer( &mxhouse.stream, InputBuffer, InputBufferPos + madguard );
      mxhouse.stream.error = (mad_error)0;
      mad_stream_sync(&mxhouse.stream);
      if ((mxstat.flushed) && (flushcnt == 2))
      {
        int skip;
        skip = 2;
        do
        {
          if (mad_frame_decode(&mxhouse.frame, &mxhouse.stream) == 0)
          {
            if (--skip == 0)
              mad_synth_frame(&mxhouse.synth, &mxhouse.frame);
          }
          else if (!MAD_RECOVERABLE(mxhouse.stream.error))
            break;
        }
        while (skip);
        mxstat.flushed = false;
      }
    }
    int maxtowrite = *out_len;
    *out_len = 0;
    mxsig = ERROR_OCCURED;
    while ((mxsig != FLUSH_BUFFER) && (*out_len + mxstat.framepcmsize < (size_t)maxtowrite))
    {
      mxsig = madx_read(&mxhouse, &mxstat, maxtowrite);
      switch (mxsig)
      {
        case ERROR_OCCURED: 
          *out_len = 0;
          HaveData = false;
          return -1;
        case MORE_INPUT: 
          if (mxstat.remaining > 0)
          {
            memcpy(InputBuffer, mxhouse.stream.next_frame, mxstat.remaining);
            InputBufferPos = mxstat.remaining;
          }
          HaveData = false;
          return 0;
        case FLUSH_BUFFER:
          FormatData[2] = mxhouse.synth.pcm.channels;
          FormatData[1] = mxhouse.synth.pcm.samplerate;
          FormatData[3] = BITSPERSAMPLE;
          FormatData[4] = mxhouse.frame.header.bitrate;
          *out_len += (int)mxstat.write_size;
          mxstat.write_size = 0;
          break;
        default:
          break;
      }
    }
    if (!mxhouse.stream.next_frame || (mxhouse.stream.bufend - mxhouse.stream.next_frame <= 0))
    {
      HaveData = false;
      return 0;
    }
    HaveData = true;
    return 1;
  }

  int Read(int size, bool init)
  {
    size_t inputBufferToRead = 65536 - InputBufferPos;
    if ( inputBufferToRead && !CallAgainWithSameBuffer && !eof )
    {
      if (XBMC->GetFileLength(file) > 0)
      {
        int fileLeft=(int)(XBMC->GetFileLength(file) - XBMC->GetFilePosition(file));
        if (inputBufferToRead >  fileLeft)
          inputBufferToRead = fileLeft;
      }

      size_t dwBytesRead = XBMC->ReadFile(file, &InputBuffer[0] + InputBufferPos , inputBufferToRead);
      if (!dwBytesRead)
      {
        XBMC->Log(ADDON::LOG_ERROR, "MP3Codec: Error reading file");
        return DECODING_ERROR;
      }
      // add the size of read PAP data to the buffer size
      InputBufferPos += dwBytesRead;
      if (XBMC->GetFileLength(file) > 0 && XBMC->GetFileLength(file) == XBMC->GetFilePosition(file))
        eof = true;
    }
    // Decode data if we have some to decode
    if (InputBufferPos || CallAgainWithSameBuffer || (eof && Decoding))
    {
      Decoding = true;

      if ( size )
      {
        CallAgainWithSameBuffer = false;
        int outputsize = 4*OUTPUTFRAMESIZE - OutputBufferPos;
        // See if there is an ID3v2 tag at the beginning of the stream.
        // For file-based internet streams (i.e UPnP/HTTP), it is very likely to happen.
        // If we don't skip it, we may never be able to snyc to the MPEG stream
        if (init)
        {
          // Check for an ID3v2 tag header
          unsigned int tagSize = IsID3v2Header(InputBuffer,InputBufferPos);
          if(tagSize)
          {
            if (tagSize != XBMC->SeekFile(file, tagSize, SEEK_SET))
              return DECODING_ERROR;

            // Reset the read state before we return
            InputBufferPos = 0;
            CallAgainWithSameBuffer = false;

            // Please try your call again later...
            return DECODING_CALLAGAIN;
          }
        }

        // Now decode data into the vacant frame buffer.
        int result = Decode(&outputsize);
        if ( result != DECODING_ERROR)
        {
          if (init)
          {
            if (result == 0 && readRetries-- > 0)
              return Read(size,init);
            // Make sure some data was decoded. Without a valid frame, we cannot determine the audio format
            if (!outputsize)
              return DECODING_ERROR;
          }

          // let's check if we need to ignore the decoded data.
          if ( IgnoreFirst && outputsize && iFirstSample )
          {
            // starting up - lets ignore the first (typically 576) samples
            int iDelay = DECODER_DELAY + iFirstSample;  // decoder delay + encoder delay
            iDelay *= FormatData[2] * (FormatData[3] >> 3);            // sample size
            if (outputsize + IgnoredBytes >= iDelay)
            {
              // have enough data to ignore - let's move the valid data to the start
              int iAmountToMove = outputsize + IgnoredBytes - iDelay;
              memmove(OutputBuffer, OutputBuffer + outputsize - iAmountToMove, iAmountToMove);
              outputsize = iAmountToMove;
              IgnoreFirst = false;
              IgnoredBytes = 0;
            }
            else
            { // not enough data yet - ignore all of this
              IgnoredBytes += outputsize;
              outputsize = 0;
            }
          }

          // Do we still have data in the buffer to decode?
          if ( result == DECODING_CALLAGAIN )
            CallAgainWithSameBuffer = true;
          else
          { // There are no more complete frames in the input buffer
            //m_InputBufferPos = 0;
            // Check for the end of file (as we need to remove data from the end of the track)
            if (eof)
            {
              Decoding = false;
              // EOF reached - let's remove any unused samples from our frame buffers
              if (IgnoreLast && iLastSample)
              {
                unsigned int samplestoremove = (iLastSample - DECODER_DELAY);
                samplestoremove *= FormatData[2] * (FormatData[3] >> 3);            // sample size
                if (samplestoremove > OutputBufferPos)
                  samplestoremove = OutputBufferPos;
                OutputBufferPos -= samplestoremove;
                IgnoreLast = false;
              }
            }
          }
          OutputBufferPos += outputsize;
        }
        return result;
      }
    }
    readRetries = 5;
    return DECODING_SUCCESS;
  }

  /* check if 'head' is a valid mp3 frame header and return the framesize if it is (0 otherwise) */
  int IsMp3FrameHeader(unsigned long head)
  {
    const long freqs[9] = { 44100, 48000, 32000,
      22050, 24000, 16000 ,
      11025 , 12000 , 8000 };
    
    const int tabsel_123[2][3][16] = {
      { {128,32,64,96,128,160,192,224,256,288,320,352,384,416,448,},
        {128,32,48,56, 64, 80, 96,112,128,160,192,224,256,320,384,},
        {128,32,40,48, 56, 64, 80, 96,112,128,160,192,224,256,320,} },
      
      { {128,32,48,56,64,80,96,112,128,144,160,176,192,224,256,},
        {128,8,16,24,32,40,48,56,64,80,96,112,128,144,160,},
        {128,8,16,24,32,40,48,56,64,80,96,112,128,144,160,} }
    };
    
    
    if ((head & SYNC_MASK) != (unsigned long)SYNC_MASK) /* bad sync? */
      return 0;
    if ((head & VERSION_MASK) == (1 << 19)) /* bad version? */
      return 0;
    if (!(head & LAYER_MASK)) /* no layer? */
      return 0;
    if ((head & BITRATE_MASK) == BITRATE_MASK) /* bad bitrate? */
      return 0;
    if (!(head & BITRATE_MASK)) /* no bitrate? */
      return 0;
    if ((head & SAMPLERATE_MASK) == SAMPLERATE_MASK) /* bad sample rate? */
      return 0;
    if (((head >> 19) & 1) == 1 &&
        ((head >> 17) & 3) == 3 &&
        ((head >> 16) & 1) == 1)
      return 0;
    if ((head & 0xffff0000) == 0xfffe0000)
      return 0;
    
    int srate = 0;
    if(!((head >> 20) &  1))
      srate = 6 + ((head>>10)&0x3);
    else
      srate = ((head>>10)&0x3) + ((1-((head >> 19) &  1)) * 3);
    
    int framesize = tabsel_123[1 - ((head >> 19) &  1)][(4-((head>>17)&3))-1][((head>>12)&0xf)]*144000/(freqs[srate]<<(1 - ((head >> 19) &  1)))+((head>>9)&0x1);
    return framesize;
  }

  bool ReadLAMETagInfo(uint8_t* b)
  {
    if (b[0x9c] != 'L' ||
        b[0x9d] != 'A' ||
        b[0x9e] != 'M' ||
        b[0x9f] != 'E')
      return false;
    
    // Found LAME tag - extract the start and end offsets
    int iDelay = ((b[0xb1] & 0xFF) << 4) + ((b[0xb2] & 0xF0) >> 4);
    iDelay += 1152; // This header is going to be decoded as a silent frame
    int iPadded = ((b[0xb2] & 0x0F) << 8) + (b[0xb3] & 0xFF);
    iFirstSample = iDelay;
    iLastSample = iPadded;
    
    return true;
  }

  // \brief Check to see if the specified buffer contains an ID3v2 tag header
  // \param pBuf Pointer to the buffer to be examined. Must be at least 10 bytes long.
  // \param bufLen Size of the buffer pointer to by pBuf. Must be at least 10.
  // \return The length of the ID3v2 tag (including the header) if one is present, otherwise 0
  unsigned int IsID3v2Header(unsigned char* pBuf, size_t bufLen)
  {
    unsigned int tagLen = 0;
    if (bufLen < 10 || pBuf[0] != 'I' || pBuf[1] != 'D' || pBuf[2] != '3')
      return 0; // Buffer is too small for complete header, or no header signature detected

    // Retrieve the tag size (including this header)
    tagLen = UNSYNC(pBuf[6], pBuf[7], pBuf[8], pBuf[9]) + 10;

    if (pBuf[5] & 0x10) // Header is followed by a footer
      tagLen += 10; //Add footer size

    return tagLen;
  }

  //TODO: merge duplicate, but slitely different implemented) code and consts in IsMp3FrameHeader(above) and ReadDuration (below).
  // Inspired by http://rockbox.haxx.se/ and http://www.xs4all.nl/~rwvtveer/scilla
  int ReadDuration()
  {
#define SCANSIZE  8192
#define CHECKNUMFRAMES 5
#define ID3V2HEADERSIZE 10

    unsigned char* xing;
    unsigned char* vbri;
    unsigned char buffer[SCANSIZE + 1];

    const int freqtab[][4] =
    {
      {11025, 12000, 8000, 0}
      ,   /* MPEG version 2.5 */
      {44100, 48000, 32000, 0},  /* MPEG Version 1 */
      {22050, 24000, 16000, 0},  /* MPEG version 2 */
    };

    /* Check if the file has an ID3v1 tag */
    XBMC->SeekFile(file, -128, SEEK_END);
    XBMC->ReadFile(file, buffer, 3);

    bool hasid3v1=false;
    if (buffer[0] == 'T' &&
        buffer[1] == 'A' &&
        buffer[2] == 'G')
    {
      hasid3v1=true;
    }

    /* Check if the file has an ID3v2 tag (or multiple tags) */
    unsigned int id3v2Size = 0;
    XBMC->SeekFile(file, 0, SEEK_SET);
    XBMC->ReadFile(file, buffer, ID3V2HEADERSIZE);
    unsigned int size = IsID3v2Header(buffer, ID3V2HEADERSIZE);
    while (size)
    {
      id3v2Size += size;
      if (id3v2Size != XBMC->SeekFile(file, id3v2Size, SEEK_SET))
        return 0;
      if (ID3V2HEADERSIZE != XBMC->ReadFile(file, buffer, ID3V2HEADERSIZE))
        return 0;
      size = IsID3v2Header(buffer, ID3V2HEADERSIZE);
    }

    //skip any padding
    //already read ID3V2HEADERSIZE bytes so take it into account
    int iScanSize = XBMC->ReadFile(file, buffer + ID3V2HEADERSIZE, SCANSIZE - ID3V2HEADERSIZE) + ID3V2HEADERSIZE;
    int iBufferDataStart;
    do
    {
      iBufferDataStart = -1;
      for(int i = 0; i < iScanSize; i++)
      {
        //all 0x00's after id3v2 tag until first mpeg-frame are padding
        if (buffer[i] != 0)
        {
          iBufferDataStart = i;
          break;
        }
      }
      if (iBufferDataStart == -1)
      {
        id3v2Size += iScanSize;
        iScanSize = XBMC->ReadFile(file, buffer, SCANSIZE);
      }
      else
      {
        id3v2Size += iBufferDataStart;
      }
    } while (iBufferDataStart == -1 && iScanSize > 0);
    if (iScanSize <= 0)
      return 0;
    if (iBufferDataStart > 0)
    {
      //move data to front of buffer
      iScanSize -= iBufferDataStart;
      memcpy(buffer, buffer + iBufferDataStart, iScanSize);
      //fill remainder of buffer with new data
      iScanSize += XBMC->ReadFile(file, buffer + iScanSize, SCANSIZE - iScanSize);
    }

    int firstFrameOffset = id3v2Size;

    //raw mp3Data = FileSize - ID3v1 tag - ID3v2 tag
    int nMp3DataSize = (int)XBMC->GetFileLength(file) - id3v2Size;
    if (hasid3v1)
      nMp3DataSize -= 128;

    //*** find the first frame in the buffer, we do this by checking if the calculated framesize leads to the next frame a couple of times.
    //the first frame that leads to a valid next frame a couple of times is where we should start decoding.
    int firstValidFrameLocation = 0;
    for(int i = 0; i + 3 < iScanSize; i++)
    {
      int j = i;
      int framesize = 1;
      int numFramesCheck = 0;

      for (numFramesCheck = 0; (numFramesCheck < CHECKNUMFRAMES) && framesize; numFramesCheck++)
      {
        unsigned long mpegheader = (unsigned long)(
                                                   ( (buffer[j] & 255) << 24) |
                                                   ( (buffer[j + 1] & 255) << 16) |
                                                   ( (buffer[j + 2] & 255) << 8) |
                                                   ( (buffer[j + 3] & 255) )
                                                   );
        framesize = IsMp3FrameHeader(mpegheader);

        j += framesize;

        if ((j + 4) >= iScanSize)
        {
          //no valid frame found in buffer
          firstValidFrameLocation = -1;
          break;
        }
      }

      if (numFramesCheck == CHECKNUMFRAMES)
      { //found it
        firstValidFrameLocation = i;
        break;
      }
    }

    if (firstValidFrameLocation != -1)
    {
      firstFrameOffset += firstValidFrameLocation;
      nMp3DataSize -= firstValidFrameLocation;
    }
    //*** done finding first valid frame

    //find lame/xing info
    int frequency = 0, bitrate = 0, bittable = 0;
    int frame_count = 0;
    double tpf = 0.0;
    for (int i = 0; i < iScanSize; i++)
    {
      unsigned long mpegheader = (unsigned long)(
                                                 ( (buffer[i] & 255) << 24) |
                                                 ( (buffer[i + 1] & 255) << 16) |
                                                 ( (buffer[i + 2] & 255) << 8) |
                                                 ( (buffer[i + 3] & 255) )
                                                 );
      
      // Do we have a Xing header before the first mpeg frame?
      if (buffer[i ] == 'X' &&
          buffer[i + 1] == 'i' &&
          buffer[i + 2] == 'n' &&
          buffer[i + 3] == 'g')
      {
        if (buffer[i + 7] & VBR_FRAMES_FLAG) /* Is the frame count there? */
        {
          frame_count = BYTES2INT(buffer[i + 8], buffer[i + 8 + 1], buffer[i + 8 + 2], buffer[i + 8 + 3]);
          if (buffer[i + 7] & VBR_TOC_FLAG)
          {
            int iOffset = i + 12;
            if (buffer[i + 7] & VBR_BYTES_FLAG)
            {
              nMp3DataSize = BYTES2INT(buffer[i + 12], buffer[i + 12 + 1], buffer[i + 12 + 2], buffer[i + 12 + 3]);
              iOffset += 4;
            }
            float *offset = new float[101];
            for (int j = 0; j < 100; j++)
              offset[j] = (float)buffer[iOffset + j]/256.0f * nMp3DataSize + firstFrameOffset;
            offset[100] = (float)nMp3DataSize + firstFrameOffset;
            SetOffsets(100, offset);
            delete[] offset;
          }
        }
      }
      /*else if (buffer[i] == 'I' && buffer[i + 1] == 'n' && buffer[i + 2] == 'f' && buffer[i + 3] == 'o') //Info is used when CBR
       {
       //should we do something with this?
       }*/
      
      if (
          (i == firstValidFrameLocation) ||
          (
           (firstValidFrameLocation == -1) &&
           (IsMp3FrameHeader(mpegheader))
           )
          )
      {
        // skip mpeg header
        i += 4;
        int version = 0;
        /* MPEG Audio Version */
        switch (mpegheader & VERSION_MASK)
        {
          case 0:
            /* MPEG version 2.5 is not an official standard */
            version = MPEG_VERSION2_5;
            bittable = MPEG_VERSION2 - 1; /* use the V2 bit rate table */
            break;

          case (1 << 19):
            return 0;

          case (2 << 19):
            /* MPEG version 2 (ISO/IEC 13818-3) */
            version = MPEG_VERSION2;
            bittable = MPEG_VERSION2 - 1;
            break;
            
          case (3 << 19):
            /* MPEG version 1 (ISO/IEC 11172-3) */
            version = MPEG_VERSION1;
            bittable = MPEG_VERSION1 - 1;
            break;
        }

        int layer = 0;
        switch (mpegheader & LAYER_MASK)
        {
          case (3 << 17):  // LAYER_I
            layer = 1;
            break;
          case (2 << 17):  // LAYER_II
            layer = 2;
            break;
          case (1 << 17):  // LAYER_III
            layer = 3;
            break;
        }

        /* Table of bitrates for MP3 files, all values in kilo.
         * Indexed by version, layer and value of bit 15-12 in header.
         */
        const int bitrate_table[2][4][16] =
        {
          {
            {0},
            {0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448, 0},
            {0, 32, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 384, 0},
            {0, 32, 40, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 0}
          },
          {
            {0},
            {0, 32, 48, 56, 64, 80, 96, 112, 128, 144, 160, 176, 192, 224, 256, 0},
            {0, 8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160, 0},
            {0, 8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160, 0}
          }
        };

        /* Bitrate */
        int bitindex = (mpegheader & 0xf000) >> 12;
        int freqindex = (mpegheader & 0x0C00) >> 10;
        bitrate = bitrate_table[bittable][layer][bitindex];

        double tpfbs[] = { 0, 384.0f, 1152.0f, 1152.0f };
        frequency = freqtab[version][freqindex];

        if (frequency == 0)
          return 0;

        tpf = tpfbs[layer] / (double) frequency;
        if (version == MPEG_VERSION2_5 || version == MPEG_VERSION2)
          tpf /= 2;

        /* Channel mode (stereo/mono) */
        int chmode = (mpegheader & 0xc0) >> 6;
        /* calculate position of Xing VBR header */
        if (version == MPEG_VERSION1)
        {
          if (chmode == 3) /* mono */
            xing = buffer + i + 17;
          else
            xing = buffer + i + 32;
        }
        else
        {
          if (chmode == 3) /* mono */
            xing = buffer + i + 9;
          else
            xing = buffer + i + 17;
        }

        /* calculate position of VBRI header */
        vbri = buffer + i + 32;

        // Do we have a Xing header
        if (xing[0] == 'X' &&
            xing[1] == 'i' &&
            xing[2] == 'n' &&
            xing[3] == 'g')
        {
          if (xing[7] & VBR_FRAMES_FLAG) /* Is the frame count there? */
          {
            frame_count = BYTES2INT(xing[8], xing[8 + 1], xing[8 + 2], xing[8 + 3]);
            if (xing[7] & VBR_TOC_FLAG)
            {
              int iOffset = 12;
              if (xing[7] & VBR_BYTES_FLAG)
              {
                nMp3DataSize = BYTES2INT(xing[12], xing[12 + 1], xing[12 + 2], xing[12 + 3]);
                iOffset += 4;
              }
              float *offset = new float[101];
              for (int j = 0; j < 100; j++)
                offset[j] = (float)xing[iOffset + j]/256.0f * nMp3DataSize + firstFrameOffset;
              //first offset should be the location of the first frame, usually it is but some files have seektables that are a little off.
              offset[0]   = (float)firstFrameOffset;
              offset[100] = (float)nMp3DataSize + firstFrameOffset;
              SetOffsets(100, offset);
              delete[] offset;
            }
          }
        }
        // Get the info from the Lame header (if any)
        if ((xing[0] == 'X' && xing[1] == 'i' && xing[2] == 'n' && xing[3] == 'g') ||
            (xing[0] == 'I' && xing[1] == 'n' && xing[2] == 'f' && xing[3] == 'o'))
        {
          if (ReadLAMETagInfo(xing - 0x24))
          {
            // calculate new (more accurate) duration:
            int64_t lastSample = (int64_t)frame_count * (int64_t)tpfbs[layer] - iFirstSample - iLastSample;
            fTotalDuration = (float)lastSample / frequency;
          }
        }
        if (vbri[0] == 'V' &&
            vbri[1] == 'B' &&
            vbri[2] == 'R' &&
            vbri[3] == 'I')
        {
          frame_count = BYTES2INT(vbri[14], vbri[14 + 1],
                                  vbri[14 + 2], vbri[14 + 3]);
          nMp3DataSize = BYTES2INT(vbri[10], vbri[10 + 1], vbri[10 + 2], vbri[10 + 3]);
          int iSeekOffsets = (((vbri[18] & 0xFF) << 8) | (vbri[19] & 0xFF)) + 1;
          float *offset = new float[iSeekOffsets + 1];
          int iScaleFactor = ((vbri[20] & 0xFF) << 8) | (vbri[21] & 0xFF);
          int iOffsetSize = ((vbri[22] & 0xFF) << 8) | (vbri[23] & 0xFF);
          offset[0] = (float)firstFrameOffset;
          for (int j = 0; j < iSeekOffsets; j++)
          {
            size_t dwOffset = 0;
            for (int k = 0; k < iOffsetSize; k++)
            {
              dwOffset = dwOffset << 8;
              dwOffset += vbri[26 + j*iOffsetSize + k];
            }
            offset[j] += (float)dwOffset * iScaleFactor;
            offset[j + 1] = offset[j];
          }
          offset[iSeekOffsets] = (float)firstFrameOffset + nMp3DataSize;
          SetOffsets(iSeekOffsets, offset);
          delete[] offset;
        }
        // We are done!
        break;
      }
    }

    if (SeekOffset.size() == 0)
    {
      float offset[2];
      offset[0] = (float)firstFrameOffset;
      offset[1] = (float)(firstFrameOffset + nMp3DataSize);
      SetOffsets(1, offset);
    }

    // Calculate duration if we have a Xing/VBRI VBR file
    if (frame_count > 0)
    {
      double d = tpf * frame_count;
      fTotalDuration = (float)d;
      return (int)d;
    }

    // Normal mp3 with constant bitrate duration
    // Now song length is (filesize without id3v1/v2 tag)/((bitrate)/(8))
    double d = 0;
    if (bitrate > 0)
      d = (double)(nMp3DataSize / ((bitrate * 1000) / 8));
    fTotalDuration = (float)d;
    return (int)d;
  }

};

void* Init(const char* strFile, unsigned int filecache, int* channels,
           int* samplerate, int* bitspersample, int64_t* totaltime,
           int* bitrate, AEDataFormat* format, const AEChannel** channelinfo)
{
  *bitrate = 0;

  MADContext* result = new MADContext;
  result->file = XBMC->OpenFile(strFile, 0);
  if (!result->file)
  {
    XBMC->Log(ADDON::LOG_ERROR, "MP3Codec: Unable to open file %s", strFile);
    delete result;
    return NULL;
  }

  int64_t length = XBMC->GetFileLength(result->file);
  if (length != 0)
  {
    int gpos = result->ReadDuration();
    *totaltime = result->fTotalDuration*1000.0f;
    if (result->SeekOffset.size() > 0 && gpos)
    {
      XBMC->SeekFile(result->file, (int)result->SeekOffset[0], SEEK_SET);
      if (*totaltime && length-result->SeekOffset[0] > 0)
        *bitrate = (length-result->SeekOffset[0])/result->fTotalDuration*8;
    }
  }

  int ir=DECODING_ERROR;
  result->eof=false;
  while (ir != DECODING_SUCCESS && !result->eof && result->OutputBufferPos < OUTPUTFRAMESIZE)
  {
    int size=0;
    ir = result->Read(8192, true);
    if (ir == DECODING_ERROR)
    {
      XBMC->Log(ADDON::LOG_ERROR, "MP3Codec: Unable to determine file format of %s (corrupt start of mp3?)", strFile);
      XBMC->CloseFile(result->file);
      delete result;
      return NULL;
    }
  }

  if (*bitrate == 0)
    *bitrate = result->FormatData[4];

  *channels = result->FormatData[2];
  *samplerate = result->FormatData[1];
  *bitspersample = result->FormatData[3];
  *format = AE_FMT_FLOAT;

  static enum AEChannel map[2][3] = {
    {AE_CH_FC, AE_CH_NULL},
    {AE_CH_FL, AE_CH_FR  , AE_CH_NULL}
  };

  if (*channels > 0 && *channels < 3)
    *channelinfo = map[*channels-1];
  else
    *channelinfo = 0;

  return result;
}

int ReadPCM(void* context, uint8_t* pBuffer, int size, int *actualsize)
{
  if (!context)
    return 1;

  MADContext* ctx = (MADContext*)context;

  *actualsize = 0;
  if (ctx->Read(size, false) == DECODING_ERROR)
    return 1;

  // check whether we can move data out of our output buffer
  // we leave some data in our output buffer to allow us to remove samples
  // at the end of the track for gapless playback
  int move;
  if ((ctx->eof && !ctx->Decoding) || ctx->OutputBufferPos <= OUTPUTFRAMESIZE)
    move = ctx->OutputBufferPos;
  else
    move = ctx->OutputBufferPos - OUTPUTFRAMESIZE;
  move = std::min(move, size);

  memcpy(pBuffer, ctx->OutputBuffer, move);
  ctx->OutputBufferPos -= move;
  memmove(ctx->OutputBuffer, ctx->OutputBuffer + move, ctx->OutputBufferPos);
  *actualsize = move;

  // only return READ_EOF when we've reached the end of the mp3 file, we've finished decoding, and our output buffer is depleated.
  if (ctx->eof && !ctx->Decoding && !ctx->OutputBufferPos)
    return -1;

  return 0;
}

int64_t Seek(void* context, int64_t time)
{
  if (!context)
    return 1;

  MADContext* ctx = (MADContext*)context;
  int64_t offset = ctx->GetByteOffset(ctx, 0.001f*time);
  XBMC->SeekFile(ctx->file, offset, SEEK_SET);
  ctx->Flush();

  return time;
}

bool DeInit(void* context)
{
  MADContext* ctx = (MADContext*)context;
  XBMC->CloseFile(ctx->file);
  mad_synth_finish(&ctx->mxhouse.synth);
  mad_frame_finish(&ctx->mxhouse.frame);
  mad_stream_finish(&ctx->mxhouse.stream);
  delete ctx;

  return true;
}

bool ReadTag(const char* strFile, char* title, char* artist,
             int* length)
{
  return true;
}

int TrackCount(const char* strFile)
{
  return 1;
}
}
