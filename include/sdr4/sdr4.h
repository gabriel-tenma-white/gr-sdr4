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


#ifndef INCLUDED_XAXAXA_SDR4_H
#define INCLUDED_XAXAXA_SDR4_H

#include <sdr4/api.h>
#include <gnuradio/sync_block.h>
#include <string>
using namespace std;

namespace gr {
  namespace sdr4 {

    /*!
     * \brief <+description of block+>
     * \ingroup sdr4
     *
     */
    class XAXAXA_API sdr4 : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<sdr4> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of sdr4::sdr4.
       *
       * To avoid accidental use of raw pointers, sdr4::sdr4's
       * constructor is in a private implementation
       * class. sdr4::sdr4::make is the public interface for
       * creating new instances.
       */
      static sptr make(string file);
    };

  } // namespace sdr4
} // namespace gr

#endif /* INCLUDED_XAXAXA_SDR4_H */

