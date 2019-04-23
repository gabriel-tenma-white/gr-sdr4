/* -*- c++ -*- */
/* 
 * Copyright 2018 <+YOU OR YOUR COMPANY+>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "sdr4_impl.h"
#include <stdio.h>

#include <unistd.h>
#include <assert.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <poll.h>
#include <string.h>

typedef int16_t s16;
typedef int32_t s32;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint8_t u8;
typedef uint64_t u64;

#ifdef assert
#undef assert
#endif
#define assert(x) ((void)(!(x) && myassert_handler(#x, __FILE__, __LINE__)))

bool myassert_handler(const char* expr, const char* file, int line) {
  fprintf(stderr, "assertion %s failed in %s:%d\n", expr, file, line);
  fflush(stderr);
  _exit(10);
  return true;
}

namespace gr {
  namespace sdr4 {
    void drainfd(int fd) {
      pollfd pfd;
      pfd.fd = fd;
      pfd.events = POLLIN;
      while(poll(&pfd,1,0)>0) {
        if(!(pfd.revents&POLLIN)) continue;
        char buf[4096];
        read(fd,buf,sizeof(buf));
      }
    }

    int writeAll(int fd,void* buf, int len) {
      u8* buf1=(u8*)buf;
      int off=0;
      int r;
      while(off<len) {
        if((r=write(fd,buf1+off,len-off))<=0) break;
        off+=r;
      }
      return off;
    }

    int readAll(int fd,void* buf, int len) {
      u8* buf1=(u8*)buf;
      int off=0;
      int r;
      while(off<len) {
        if((r=read(fd,buf1+off,len-off))<=0) break;
        off+=r;
      }
      return off;
    }

    sdr4::sptr
    sdr4::make(string file)
    {
      return gnuradio::get_initial_sptr
        (new sdr4_impl(file));
    }

    /*
     * The private constructor
     */
    sdr4_impl::sdr4_impl(string file)
      : gr::sync_block("sdr4",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(1, 1, sizeof(complex<float>)))
    {
      ttyFD = open("/dev/ttyACM0", O_RDWR);
      if(ttyFD<0) throw runtime_error(strerror(errno));
      
      struct termios tc;
      /* Set TTY mode. */
      if (tcgetattr(ttyFD, &tc) < 0) {
        perror("tcgetattr");
        return;
      }
      tc.c_iflag &= ~(INLCR|IGNCR|ICRNL|IGNBRK|IUCLC|INPCK|ISTRIP|IXON|IXOFF|IXANY);
      tc.c_oflag &= ~OPOST;
      tc.c_cflag &= ~(CSIZE|CSTOPB|PARENB|PARODD|CRTSCTS);
      tc.c_cflag |= CS8 | CREAD | CLOCAL;
      tc.c_lflag &= ~(ICANON|ECHO|ECHOE|ECHOK|ECHONL|ISIG|IEXTEN);
      tc.c_cc[VMIN] = 1;
      tc.c_cc[VTIME] = 0;
      if (tcsetattr(ttyFD, TCSANOW, &tc) < 0) {
        perror("tcsetattr");
      }
      outBufSize = 1024*32;
      outBuf = new uint32_t[outBufSize];
      buf = new uint8_t[outBufSize*3];
    }

    /*
     * Our virtual destructor.
     */
    sdr4_impl::~sdr4_impl()
    {
      delete[] outBuf;
      delete[] buf;
    }

    int
    sdr4_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      complex<float>* out = (complex<float>*) output_items[0];

      // Do <+signal processing+>
      //int ret = readAll(ttyFD, out, noutput_items*sizeof(*out));
      
      assert(noutput_items <= outBufSize);
      
      int bufLen = noutput_items;
      int outBufPos = 0;
      
      memset(buf, 0, bufLen);
      bufLen = readAll(ttyFD,buf,bufLen);
      for(int i=0;i<bufLen;i++) {
        curr = buf[i];
        if(curr == 0xdd && prev1 == 0xbe && prev2 == 0xef) {
          state = 0;
          dat = 0;
          goto cont;
        }
        if(state == 0) dat = 0;
        dat |= (u32(curr) << (state*8));
        
        state++;
        if(state>2) {
          outBuf[outBufPos++] = dat;
          state=0;
          dat=0;
        }
      cont:
        prev2=prev1;
        prev1=curr;
      }
      
      for(int i=0;i<outBufPos;i++) {
        s16 I = outBuf[i]&0b111111111111;
        s16 Q = (outBuf[i]>>12)&0b111111111111;
        I <<= 4; I >>= 4;
        Q <<= 4; Q >>= 4;
        out[i] = complex<float>(float(I)*(1./2048), float(Q)*(1./2048));
        //out[i] = complex<float>(float(int8_t(buf[i]))*.01, 0);
      }
      // Tell runtime system how many output items we produced.
      return outBufPos;
    }

  } /* namespace sdr4 */
} /* namespace gr */

