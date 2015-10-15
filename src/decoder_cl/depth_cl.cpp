/*
 * This file is a part of the OpenKinect project
 *
 * Copyright (c) 2014 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <assert.h>

#if defined(WIN32)
#define _USE_MATH_DEFINES
#endif

#include <math.h>

#define __CL_ENABLE_EXCEPTIONS
#if defined(__APPLE__ ) || defined(__MACOSX)
#include <OpenCL/cl.hpp>
#else
#include <CL/cl.h>
#include <CL/cl.hpp>
#endif

#include "module.h"
#include "libk4w2/kinect2.h"

#define IMAGE_SIZE (512 * 424)

const size_t buf_packet_size = KINECT2_DEPTH_FRAME_SIZE * 10;

const size_t buf_a_size = IMAGE_SIZE * sizeof(cl_float3);
const size_t buf_b_size = IMAGE_SIZE * sizeof(cl_float3);
const size_t buf_n_size = IMAGE_SIZE * sizeof(cl_float3);
const size_t buf_ir_size = IMAGE_SIZE * sizeof(cl_float);
const size_t buf_depth_size = IMAGE_SIZE * sizeof(cl_float);
const size_t buf_ir_sum_size = IMAGE_SIZE * sizeof(cl_float);

struct parameters {
     float ab_multiplier;
     float ab_multiplier_per_frq[3];
     float ab_output_multiplier;

     float phase_in_rad[3];

     float phase_offset;
     float unambigious_dist;
     float individual_ab_threshold;
     float ab_threshold;
     float ab_confidence_slope;
     float ab_confidence_offset;
     float min_dealias_confidence;
     float max_dealias_confidence;

     float min_depth;
     float max_depth;
};

/**
 * @class DecoderCL depth_cl.cpp
 *
 */
class DecoderCL
{
public:
    bool setup(const parameters &params, size_t num_slot);
    int set_params(const kinect2_color_camera_param * color,
		   const kinect2_depth_camera_param * depth,
		   const kinect2_p0table * p0table);

    bool request(int slot, const void *ptr, int length);
    bool fetch(int slot, void *dst, int dst_length);
private:
    cl::Context context;
    cl::CommandQueue queue;

    cl::Program program;

    // Read only buffers
    cl::Buffer buf_lut11to16;
    cl::Buffer buf_p0_table;
    cl::Buffer buf_x_table;
    cl::Buffer buf_z_table;

    struct Slot {
	cl::Kernel kernel_processPixelStage1;
	cl::Kernel kernel_processPixelStage2;

	cl::Buffer buf_packet;

	cl::Buffer buf_a;
	cl::Buffer buf_b;
	cl::Buffer buf_n;
	cl::Buffer buf_ir;
	cl::Buffer buf_depth;
	cl::Buffer buf_ir_sum;

	std::vector<cl::Event> eventWrite, eventPPS1,  eventPPS2;
	cl::Event event0, event1;

	void create(cl::Context context,
		    cl::Program program,
		    cl::Buffer buf_lut11to16,
		    cl::Buffer buf_p0_table,
		    cl::Buffer buf_z_table,
		    cl::Buffer buf_x_table);
    };
    std::vector<Slot> m_slot;
};



static void
init_params(struct parameters *p)
{
     p->ab_multiplier = 0.6666667f;
     p->ab_multiplier_per_frq[0] = 1.322581f;
     p->ab_multiplier_per_frq[1] = 1.0f;
     p->ab_multiplier_per_frq[2] = 1.612903f;
     p->ab_output_multiplier = 16.0f;

     p->phase_in_rad[0] = 0.0f;
     p->phase_in_rad[1] = 2.094395f;
     p->phase_in_rad[2] = 4.18879f;

     p->phase_offset = 0.0f;
     p->unambigious_dist = 2083.333f;
     p->individual_ab_threshold  = 3.0f;
     p->ab_threshold = 10.0f;
     p->ab_confidence_slope = -0.5330578f;
     p->ab_confidence_offset = 0.7694894f;
     p->min_dealias_confidence = 0.3490659f;
     p->max_dealias_confidence = 0.6108653f;

     p->min_depth = 500.0f;
     p->max_depth = 4500.0f;
}

static void
fill_trig_table(const kinect2_p0table *src, cl_float3 *dst)
{
     for(int r = 0; r < 424; ++r)
     {
	  cl_float3 *it = &dst[r * 512];
	  const uint16_t *it0 = &src->p0table0[r * 512];
	  const uint16_t *it1 = &src->p0table1[r * 512];
	  const uint16_t *it2 = &src->p0table2[r * 512];
	  for(int c = 0; c < 512; ++c, ++it, ++it0, ++it1, ++it2)
	  {
	       it->s[0] = -((float) * it0) * 0.000031 * M_PI;
	       it->s[1] = -((float) * it1) * 0.000031 * M_PI;
	       it->s[2] = -((float) * it2) * 0.000031 * M_PI;
	       //it->s[3] = 0.0f;
	  }
     }
}

static void
generateOptions(const parameters &params, std::string *options) 
{
     std::ostringstream oss;
     oss.precision(16);
     oss << std::scientific;
     oss << " -D KINECT2_DEPTH_FRAME_SIZE=" << KINECT2_DEPTH_FRAME_SIZE;
     oss << " -D BFI_BITMASK=" << "0x180";

     oss << " -D AB_MULTIPLIER=" << params.ab_multiplier << "f";
     oss << " -D AB_MULTIPLIER_PER_FRQ0=" << params.ab_multiplier_per_frq[0] << "f";
     oss << " -D AB_MULTIPLIER_PER_FRQ1=" << params.ab_multiplier_per_frq[1] << "f";
     oss << " -D AB_MULTIPLIER_PER_FRQ2=" << params.ab_multiplier_per_frq[2] << "f";
     oss << " -D AB_OUTPUT_MULTIPLIER=" << params.ab_output_multiplier << "f";

     oss << " -D PHASE_IN_RAD0=" << params.phase_in_rad[0] << "f";
     oss << " -D PHASE_IN_RAD1=" << params.phase_in_rad[1] << "f";
     oss << " -D PHASE_IN_RAD2=" << params.phase_in_rad[2] << "f";

     oss << " -D PHASE_OFFSET=" << params.phase_offset << "f";
     oss << " -D UNAMBIGIOUS_DIST=" << params.unambigious_dist << "f";
     oss << " -D INDIVIDUAL_AB_THRESHOLD=" << params.individual_ab_threshold << "f";
     oss << " -D AB_THRESHOLD=" << params.ab_threshold << "f";
     oss << " -D AB_CONFIDENCE_SLOPE=" << params.ab_confidence_slope << "f";
     oss << " -D AB_CONFIDENCE_OFFSET=" << params.ab_confidence_offset << "f";
     oss << " -D MIN_DEALIAS_CONFIDENCE=" << params.min_dealias_confidence << "f";
     oss << " -D MAX_DEALIAS_CONFIDENCE=" << params.max_dealias_confidence << "f";

     *options = oss.str();
}


void
DecoderCL::Slot::create(cl::Context context,
			cl::Program program,
			cl::Buffer buf_lut11to16,
			cl::Buffer buf_p0_table,
			cl::Buffer buf_x_table,
			cl::Buffer buf_z_table)
{
    cl_int err;
    buf_packet = cl::Buffer(context, CL_READ_ONLY_CACHE, buf_packet_size, NULL, &err);

    buf_a = cl::Buffer(context, CL_READ_WRITE_CACHE, buf_a_size, NULL, &err);
    buf_b = cl::Buffer(context, CL_READ_WRITE_CACHE, buf_b_size, NULL, &err);
    buf_n = cl::Buffer(context, CL_READ_WRITE_CACHE, buf_n_size, NULL, &err);
    buf_ir = cl::Buffer(context, CL_READ_WRITE_CACHE, buf_ir_size, NULL, &err);

    buf_depth = cl::Buffer(context, CL_READ_WRITE_CACHE, buf_depth_size, NULL, &err);
    buf_ir_sum = cl::Buffer(context, CL_READ_WRITE_CACHE, buf_ir_sum_size, NULL, &err);

    eventWrite.resize(1);
    eventPPS1.resize(1);
    eventPPS2.resize(1);

    kernel_processPixelStage1 = cl::Kernel(program, "processPixelStage1", &err);
    kernel_processPixelStage1.setArg(0, buf_lut11to16);
    kernel_processPixelStage1.setArg(1, buf_z_table);
    kernel_processPixelStage1.setArg(2, buf_p0_table);
    kernel_processPixelStage1.setArg(3, buf_packet);
    kernel_processPixelStage1.setArg(4, buf_a);
    kernel_processPixelStage1.setArg(5, buf_b);
    kernel_processPixelStage1.setArg(6, buf_n);
    kernel_processPixelStage1.setArg(7, buf_ir);

    kernel_processPixelStage2 = cl::Kernel(program, "processPixelStage2", &err);
    kernel_processPixelStage2.setArg(0, buf_a);
    kernel_processPixelStage2.setArg(1, buf_b);
    kernel_processPixelStage2.setArg(2, buf_x_table);
    kernel_processPixelStage2.setArg(3, buf_z_table);
    kernel_processPixelStage2.setArg(4, buf_depth);
    kernel_processPixelStage2.setArg(5, buf_ir_sum);
}

bool
DecoderCL::setup(const parameters &params,
		 size_t num_slot)
{
    cl_int err = CL_SUCCESS;
    try { 
        std::vector<cl::Platform> platforms;
        cl::Platform::get(&platforms);
        if (platforms.size() == 0) {
            std::cout << "Platform size 0\n";
            return K4W2_ERROR;
        }
        cl_context_properties properties[] = 
	    { CL_CONTEXT_PLATFORM,
	      (cl_context_properties)(platforms[0])(),
	      0};
        context = cl::Context(CL_DEVICE_TYPE_GPU, properties); 

	std::vector<cl::Device> devices = context.getInfo<CL_CONTEXT_DEVICES>();

	queue   = cl::CommandQueue(context, devices[0], 0, &err);
    }
    catch (cl::Error err) {
	std::cerr 
            << "ERROR: "
            << err.what()
            << "("
            << err.err()
            << ")"
            << std::endl;
	return false;
    }

    static const char *searchpath[] = {
	K4W2_SRCDIR"/decoder_cl",
	K4W2_DATADIR,
    };

    const int MAX_SOURCECODE_SIZE = 20 * 1024;
    char* sourcecode = new char [MAX_SOURCECODE_SIZE];
    size_t sourcelength;
    int r = k4w2_search_and_load(searchpath, ARRAY_SIZE(searchpath),
				 "depth.cl",
				 sourcecode, MAX_SOURCECODE_SIZE,
				 &sourcelength);
    if (K4W2_SUCCESS != r) {
	ABORT("failed to load ");
    }

    try
    {
	std::string options;
	generateOptions(params, &options);

	cl::Program::Sources src(1,
				 std::make_pair(sourcecode,
						sourcelength));
	program = cl::Program(context, src);
	program.build(options.c_str());

	buf_lut11to16 = cl::Buffer(context, CL_READ_ONLY_CACHE, 2*2048UL, NULL,
				   &err);
	buf_p0_table = cl::Buffer(context, CL_READ_ONLY_CACHE,
				  IMAGE_SIZE * sizeof(cl_float3), NULL,
				  &err);
	buf_x_table = cl::Buffer(context, CL_READ_ONLY_CACHE,
				 IMAGE_SIZE * sizeof(cl_float3), NULL,
				 &err);
	buf_z_table = cl::Buffer(context, CL_READ_ONLY_CACHE,
				 IMAGE_SIZE * sizeof(cl_float3), NULL,
				 &err);

	m_slot.resize(num_slot);
	for (size_t i=0; i<num_slot; ++i) {
	    m_slot[i].create(context,
			     program,
			     buf_lut11to16,
			     buf_p0_table,
			     buf_x_table,
			     buf_z_table);
	}
    }
    catch(const cl::Error &err)
    {
	VERBOSE("%s %s", err.what() , err.err());
	if(err.err() == CL_BUILD_PROGRAM_FAILURE)
	{
/*	    
	    std::cout << "Build Status: " << program.getBuildInfo<CL_PROGRAM_BUILD_STATUS>(device) << std::endl;
	    std::cout << "Build Options:\t" << program.getBuildInfo<CL_PROGRAM_BUILD_OPTIONS>(device) << std::endl;
	    std::cout << "Build Log:\t " << program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(device) << std::endl;*/
	}
	return false;
    }
    return true;
}

int
DecoderCL::set_params(const kinect2_color_camera_param * color,
		      const kinect2_depth_camera_param * depth,
		      const kinect2_p0table * p0table)
{
    static const char *searchpath[] = {
	".",
	K4W2_SRCDIR,
	K4W2_DATADIR,
    };

    cl_int err = CL_SUCCESS;
    try { 
	{
	    size_t actual_size;
	    int16_t lut[2048];
	    int r = k4w2_search_and_load(searchpath, ARRAY_SIZE(searchpath),
					 "11to16.bin",
					 lut, sizeof(lut),
					 &actual_size);
	    if (K4W2_SUCCESS != r || sizeof(lut) != actual_size)
		return r;
	    queue.enqueueWriteBuffer(buf_lut11to16, CL_TRUE,
				     0, sizeof(lut), lut,
				     NULL, NULL);
	}

	{
	    cl_float3 tmp[IMAGE_SIZE];
	    fill_trig_table(p0table, tmp);
	    queue.enqueueWriteBuffer(buf_p0_table, CL_TRUE,
				     0, sizeof(tmp), tmp,
				     NULL, NULL);
	}

	{
	    cl_float x_table[IMAGE_SIZE];
	    size_t actual_size;
	    int r = k4w2_search_and_load(searchpath, ARRAY_SIZE(searchpath),
					 "xTable.bin",
					 x_table, sizeof(x_table),
					 &actual_size);
	    if (K4W2_SUCCESS != r || sizeof(x_table) != actual_size)
		return r;
	    queue.enqueueWriteBuffer(buf_x_table, CL_TRUE,
				     0, sizeof(x_table), x_table,
				     NULL, NULL);
	}

	{
	    cl_float z_table[IMAGE_SIZE];
	    size_t actual_size;
	    int r = k4w2_search_and_load(searchpath, ARRAY_SIZE(searchpath),
					 "zTable.bin",
					 z_table, sizeof(z_table),
					 &actual_size);
	    if (K4W2_SUCCESS != r || sizeof(z_table) != actual_size)
		return r;     
	    queue.enqueueWriteBuffer(buf_z_table, CL_TRUE,
				     0, sizeof(z_table), z_table,
				     NULL, NULL);
	}
    }
    catch(const cl::Error &err)
    {
	VERBOSE("%s %s", err.what() , err.err());
    }
    return K4W2_SUCCESS;
}


bool
DecoderCL::request(int slot, const void *ptr, int length)
{
    if (KINECT2_DEPTH_FRAME_SIZE*10 != length) {
	return false;
    }

    Slot& s = m_slot[slot];
    try
    {
	queue.enqueueWriteBuffer(s.buf_packet, CL_FALSE, 0, length, ptr,
				 NULL, &s.eventWrite[0]);

	queue.enqueueNDRangeKernel(s.kernel_processPixelStage1,
				   cl::NullRange,
				   cl::NDRange(IMAGE_SIZE),
				   cl::NullRange,
				   &s.eventWrite, &s.eventPPS1[0]);

	queue.enqueueNDRangeKernel(s.kernel_processPixelStage2,
				   cl::NullRange, cl::NDRange(IMAGE_SIZE), cl::NullRange,
				   &s.eventPPS1, &s.eventPPS2[0]);
    }
    catch (const cl::Error & err) {
	std::cerr 
            << "ERROR: "
            << err.what()
            << "("
            << err.err()
            << ")"
            << std::endl;
	return false;
    }
    return true;
}

bool
DecoderCL::fetch(int slot, void *dst, int dst_length)
{
    Slot& s = m_slot[slot];
    try
    {
	queue.enqueueReadBuffer(s.buf_ir, CL_FALSE,
				0, buf_ir_size, (char*)dst + buf_depth_size,
				&s.eventPPS1, &s.event0);
	queue.enqueueReadBuffer(s.buf_depth, CL_FALSE,
				0, buf_depth_size, dst,
				&s.eventPPS2, &s.event1);
	s.event0.wait();
	s.event1.wait();
    }
    catch (const cl::Error & err) {
	std::cerr 
            << "ERROR: "
            << err.what()
            << "("
            << err.err()
            << ")"
            << std::endl;
	return false;
    }
    return true;
}


#define BEGIN_EXTERN_C extern "C" {
#define END_EXTERN_C   }


BEGIN_EXTERN_C

struct depth_cl {
    struct k4w2_decoder_ctx decoder; 
    DecoderCL dcl;
};

static int
depth_cl_open(k4w2_decoder_t ctx, unsigned int type)
{

    if ( (type & K4W2_DECODER_TYPE_MASK) != K4W2_DECODER_DEPTH)
	return K4W2_ERROR;
    if ( type & K4W2_DECODER_DISABLE_OPENCL )
	return K4W2_ERROR;

    depth_cl * d = (depth_cl *)ctx;

    // Call placement new to initialize OpenCL's variables
    // because it was allocated by malloc()
    new (&d->dcl) DecoderCL;

    parameters params;
    init_params(&params);

    if (! d->dcl.setup(params, ctx->num_slot) ) {
	return K4W2_ERROR;
    }

    return K4W2_SUCCESS;

}

static int
depth_cl_set_params(k4w2_decoder_t ctx, 
		     struct kinect2_color_camera_param * color,
		     struct kinect2_depth_camera_param * depth,
		     struct kinect2_p0table * p0table)
{
    depth_cl * d = (depth_cl *)ctx;
    return d->dcl.set_params(color, depth, p0table);
}

static int
depth_cl_request(k4w2_decoder_t ctx, int slot, const void *src, int src_length)
{
    depth_cl * d = (depth_cl *)ctx;
    bool r = d->dcl.request(slot, src, src_length);
    return r?K4W2_SUCCESS:K4W2_ERROR;
}

static int
depth_cl_fetch(k4w2_decoder_t ctx, int slot, void *dst, int dst_length)
{
    depth_cl * d = (depth_cl *)ctx;
    bool r = d->dcl.fetch(slot, dst, dst_length);
    return r?K4W2_SUCCESS:K4W2_ERROR;
}

static int
depth_cl_close(k4w2_decoder_t ctx)
{
    depth_cl * d = (depth_cl *)ctx;

    // Call destructor explicitly because it was allocated by malloc()
    d->dcl.~DecoderCL();
}

REGISTER_MODULE(k4w2_decoder_depth_cl_init)
{
    static k4w2_decoder_ops ops;
    ops.open	= depth_cl_open;
    ops.set_params = depth_cl_set_params;
    ops.request	= depth_cl_request;
    //    ops.wait	= depth_cl_wait;
    ops.fetch	= depth_cl_fetch;
    ops.close	= depth_cl_close;
    
    k4w2_register_decoder("depth OpenCL", &ops, sizeof(depth_cl));
}

END_EXTERN_C

// Local Variables:
// mode: c++
// c-basic-offset:  4
// End:
