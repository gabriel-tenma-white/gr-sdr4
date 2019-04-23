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

#ifndef INCLUDED_XAXAXA_SDR4_IMPL_H
#define INCLUDED_XAXAXA_SDR4_IMPL_H

#include <sdr4/sdr4.h>
#include <string>
#include <stdint.h>

using namespace std;
namespace gr {
  namespace sdr4 {

    class sdr4_impl : public sdr4
    {
     private:
      uint8_t* buf;
      uint32_t* outBuf;
      int outBufSize;
      
      uint8_t prev1=0, prev2=0, curr;
      int state=0;
      uint32_t dat=0;

     public:
      sdr4_impl(string file);
      ~sdr4_impl();
      int ttyFD;

      // Where all the action really happens
      int work(int noutput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
    };

  } // namespace sdr4
} // namespace gr

#endif /* INCLUDED_XAXAXA_SDR4_IMPL_H */

