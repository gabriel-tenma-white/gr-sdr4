/* -*- c++ -*- */

#define XAXAXA_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "sdr4_swig_doc.i"

%{
#include "sdr4/sdr4.h"
%}


%include "sdr4/sdr4.h"
GR_SWIG_BLOCK_MAGIC2(sdr4, sdr4);
