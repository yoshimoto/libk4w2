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
#if defined(HAVE_GLEW)
#include <GL/glew.h>
#  if defined(__APPLE__ ) || defined(__MACOSX)
#  include <OpenGL/OpenGL.h>
#  elif defined(WIN32)
#  include <windows.h> /* not checked yet. */
#  else
#  include <GL/glx.h>
#  endif
#endif

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

#define CHECK_GL() do {							\
	GLenum e;							\
	while ( (e = glGetError()) != GL_NO_ERROR ) {			\
	    VERBOSE("glGetError() returns '%s'",			\
		    glewGetErrorString(e) );				\
	}								\
    } while(0)

#define IMAGE_SIZE (512 * 424)

const size_t buf_packet_size = KINECT2_DEPTH_FRAME_SIZE * 10;
const size_t buf_a_size = IMAGE_SIZE * sizeof(cl_float3);
const size_t buf_b_size = IMAGE_SIZE * sizeof(cl_float3);
const size_t buf_n_size = IMAGE_SIZE * sizeof(cl_float3);
const size_t buf_ir_size = IMAGE_SIZE * sizeof(cl_float);
const size_t buf_depth_size = IMAGE_SIZE * sizeof(cl_float);

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
    bool setup(const parameters &params, size_t num_slot, unsigned int type);
    int set_params(const kinect2_color_camera_param * color,
		   const kinect2_depth_camera_param * depth,
		   const kinect2_p0table * p0table);

    bool request(int slot, const void *ptr, int length);
    bool fetch(int slot, void *dst, int dst_length);
    bool get_gl_texture(int slot, unsigned int option, unsigned int *texture);
private:
    cl::Context context;
    cl::CommandQueue queue;

    cl::Program program;

    // Read only buffers
    cl::Buffer buf_lut11to16;
    cl::Buffer buf_p0_table;
    cl::Buffer buf_x_table;
    cl::Buffer buf_z_table;

    friend struct Slot;
    struct Slot {
	cl::Kernel kernel_processPixelStage1;
	cl::Kernel kernel_processPixelStage2;

	cl::Buffer buf_packet;

	cl::Buffer buf_a;
	cl::Buffer buf_b;
	cl::Buffer buf_n;

#if defined(HAVE_GLEW)
	union {
	    struct {
		GLuint depth;
		GLuint ir;
	    };
	    GLuint name[2];
	} texture;
#endif

	cl::Image2D image[2]; // 0:depth, 1:ir
	std::vector<cl::Event> eventWrite, eventPPS1,  eventPPS2;
	cl::Event event0, event1;

	void create(const DecoderCL *self, size_t slot, unsigned int type);
    };
    std::vector<Slot> m_slot;

    unsigned int m_type;
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
DecoderCL::Slot::create(const DecoderCL *self, const size_t slot, const unsigned int type)
{
    cl_int err;
    buf_packet = cl::Buffer(self->context, CL_READ_ONLY_CACHE, buf_packet_size, NULL, &err);

    buf_a = cl::Buffer(self->context, CL_READ_WRITE_CACHE, buf_a_size, NULL, &err);
    buf_b = cl::Buffer(self->context, CL_READ_WRITE_CACHE, buf_b_size, NULL, &err);
    buf_n = cl::Buffer(self->context, CL_READ_WRITE_CACHE, buf_n_size, NULL, &err);

    cl::ImageFormat format;
    format.image_channel_order = CL_R;
    format.image_channel_data_type = CL_FLOAT;

#if defined(HAVE_GLEW)
    if (type & K4W2_DECODER_ENABLE_OPENGL) {
	glGenTextures(2, &texture.name[0]);
	for (size_t i = 0; i < 2; ++i) {
	    glBindTexture(GL_TEXTURE_2D, texture.name[i]);
	    glTexStorage2D(GL_TEXTURE_2D,
			   1,
			   GL_R32F,
			   512, 424);
	    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	    CHECK_GL();
	    image[i]() = clCreateFromGLTexture(self->context(), CL_MEM_WRITE_ONLY, GL_TEXTURE_2D, 0,
					       texture.name[i], &err);
	    if (CL_SUCCESS != err) {
		ABORT("failed, err: %d", err);
	    }
	}
    } else {
	for (size_t i = 0; i < 2; ++i) {
	    texture.name[i] = 0;
	    image[i] = cl::Image2D(self->context, CL_READ_WRITE_CACHE, format,
				   512, 424, 0, NULL, &err);
	}
    }
#else
    for (size_t i = 0; i < 2; ++i) {
	image[i] = cl::Image2D(self->context, CL_READ_WRITE_CACHE, format,
			       512, 424, 0, NULL, &err);
    }
#endif

    eventWrite.resize(1);
    eventPPS1.resize(1);
    eventPPS2.resize(1);

    try { 
	kernel_processPixelStage1 = cl::Kernel(self->program, "processPixelStage1");
	kernel_processPixelStage1.setArg(0, self->buf_lut11to16);
	kernel_processPixelStage1.setArg(1, self->buf_z_table);
	kernel_processPixelStage1.setArg(2, self->buf_p0_table);
	kernel_processPixelStage1.setArg(3, buf_packet);
	kernel_processPixelStage1.setArg(4, buf_a);
	kernel_processPixelStage1.setArg(5, buf_b);
	kernel_processPixelStage1.setArg(6, buf_n);
	kernel_processPixelStage1.setArg(7, image[1]);
	
	kernel_processPixelStage2 = cl::Kernel(self->program, "processPixelStage2");
	kernel_processPixelStage2.setArg(0, buf_a);
	kernel_processPixelStage2.setArg(1, buf_b);
	kernel_processPixelStage2.setArg(2, self->buf_x_table);
	kernel_processPixelStage2.setArg(3, self->buf_z_table);
	kernel_processPixelStage2.setArg(4, image[0]);
    }
    catch (cl::Error err) {
	ABORT("%s %x", err.what(), err.err());
    }
}

#if defined(__APPLE__ ) || defined(__MACOSX)
static void *get_current_cgl_share_group()
{
    // Get current CGL Context and CGL Share group
    CGLContextObj kCGLContext = CGLGetCurrentContext();
    CGLShareGroupObj kCGLShareGroup = CGLGetShareGroup(kCGLContext);
    return kCGLShareGroup;
}
#endif // #if defined(__APPLE__ ) || defined(__MACOSX)

#if defined(HAVE_GLEW)
static bool check_opengl_context(cl_device_id device_id)
{

#if defined(__APPLE__) || defined(MACOSX)
    if (!get_current_cgl_share_group()) {
	VERBOSE("No CGLContext attached.");
	return false;
    }
#elif defined(WIN32)
    if (!(wglGetCurrentContext() && wglGetCurrentDC())) {
	VERBOSE("No OpenGL context attached");
	return false;
    }
#else
    if (!(glXGetCurrentContext() && glXGetCurrentDisplay())) {
	VERBOSE("No GLX context attached.");
	return false;
    }
#endif

#if defined(__APPLE__) || defined(MACOSX)
    static const char * CL_GL_SHARING_EXT = "cl_APPLE_gl_sharing";
#else
    static const char * CL_GL_SHARING_EXT = "cl_khr_gl_sharing";
#endif

    size_t ext_size = 2*1024;
    std::vector<char> ext_string(ext_size);
    cl_int err;
    err = clGetDeviceInfo(device_id, CL_DEVICE_EXTENSIONS,
			  ext_size, &ext_string[0], &ext_size);
    if (err != CL_SUCCESS) {
	return false;
    }
    if (NULL == strcasestr(&ext_string[0], CL_GL_SHARING_EXT)) {
	VERBOSE("%s is not supported", CL_GL_SHARING_EXT);
	return false;
    }
    return true;
}

#endif /* #if defined(HAVE_GLEW) */


static void
setup_cl_context_properties(cl_context_properties prop[], size_t max_size,
			    cl::Platform platform,
			    unsigned int type)
{
    if (type & K4W2_DECODER_ENABLE_OPENGL) {
	cl_context_properties tmp[] = {
	    CL_CONTEXT_PLATFORM,
	    (cl_context_properties)(platform)(),
#if defined (HAVE_GLEW) 
#  if defined(__APPLE__) || defined (MACOSX)
	    CL_CONTEXT_PROPERTY_USE_CGL_SHAREGROUP_APPLE,
	    (cl_context_properties)get_current_cgl_share_group(),
#  elif defined(WIN32)
	    CL_GL_CONTEXT_KHR, (cl_context_properties)wglGetCurrentContext(),
	    CL_WGL_HDC_KHR,  (cl_context_properties)wglGetCurrentDC(),
#  else
	    CL_GL_CONTEXT_KHR, (cl_context_properties)glXGetCurrentContext(),
	    CL_GLX_DISPLAY_KHR, (cl_context_properties)glXGetCurrentDisplay(),
#  endif
#endif /* #if defined (HAVE_GLEW) */
	    0};
	assert(sizeof(tmp) <= max_size);
	memcpy(prop, tmp, sizeof(tmp));
    } else {
	cl_context_properties tmp[] = {
	    CL_CONTEXT_PLATFORM,
	    (cl_context_properties)(platform)(),
	    0
	};
	memcpy(prop, tmp, sizeof(tmp));
    }
}

bool
DecoderCL::setup(const parameters &params,
		 size_t num_slot,
		 const unsigned int type)
{
    m_type = type;

    cl_int err = CL_SUCCESS;
    std::vector<cl::Device> devices;
    try { 
        std::vector<cl::Platform> platforms;
        cl::Platform::get(&platforms);
        if (platforms.size() == 0) {
            std::cout << "Platform size 0\n";
            return false;
        }

        cl_context_properties properties[10];
	setup_cl_context_properties(properties, sizeof(properties),
				    platforms[0],
				    type);

        context = cl::Context(CL_DEVICE_TYPE_GPU, properties); 

	devices = context.getInfo<CL_CONTEXT_DEVICES>();

	queue   = cl::CommandQueue(context, devices[0], 0, &err);
    }
    catch (cl::Error err) {
	VERBOSE("%s %x", err.what(), err.err() );
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

#if defined(HAVE_GLEW)
    if ( m_type & K4W2_DECODER_ENABLE_OPENGL ) {
	check_opengl_context(devices[0]());
    }

#endif

    try {
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
	    m_slot[i].create(this, i, m_type);
	}
    }
    catch(const cl::Error &err)
    {
	VERBOSE("%s %x", err.what(), err.err() );
	if(err.err() == CL_BUILD_PROGRAM_FAILURE)
	{
	    std::cerr << "Build Status: " << program.getBuildInfo<CL_PROGRAM_BUILD_STATUS>(devices[0]) << std::endl;
	    std::cerr << "Build Options:\t" << program.getBuildInfo<CL_PROGRAM_BUILD_OPTIONS>(devices[0]) << std::endl;
	    std::cerr << "Build Log:\t " << program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(devices[0]) << std::endl;
	}
	ABORT("abort.");
	return false;
    }
    return true;
}

int
DecoderCL::set_params(const kinect2_color_camera_param * color,
		      const kinect2_depth_camera_param * depth,
		      const kinect2_p0table * p0table)
{
    cl_int err = CL_SUCCESS;
    try { 
	{
	    short lut[2048];
	    int r = k4w2_create_lut_table(lut, sizeof(lut));
	    if (K4W2_SUCCESS != r)
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
	    float x_table[IMAGE_SIZE];
	    float z_table[IMAGE_SIZE];
	    int r = k4w2_create_xz_table(depth,
					 x_table, sizeof(x_table),
					 z_table, sizeof(z_table));
	    if (K4W2_SUCCESS != r)
		return r;
	    queue.enqueueWriteBuffer(buf_x_table, CL_TRUE,
				     0, sizeof(x_table), x_table,
				     NULL, NULL);
	    queue.enqueueWriteBuffer(buf_z_table, CL_TRUE,
				     0, sizeof(z_table), z_table,
				     NULL, NULL);
	}
    }
    catch(const cl::Error &err)
    {
	VERBOSE("%s %x", err.what(), err.err() );
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
	std::vector<cl::Memory> objs;
	objs.push_back(s.image[0]);
	objs.push_back(s.image[1]);

	queue.enqueueWriteBuffer(s.buf_packet, CL_FALSE, 0, length, ptr,
				 NULL, &s.eventWrite[0]);

#if defined(HAVE_GLEW)
	if (m_type & K4W2_DECODER_ENABLE_OPENGL)
	    queue.enqueueAcquireGLObjects(&objs);
#endif

	queue.enqueueNDRangeKernel(s.kernel_processPixelStage1,
				   cl::NullRange,
				   cl::NDRange(IMAGE_SIZE),
				   cl::NullRange,
				   &s.eventWrite, &s.eventPPS1[0]);

	queue.enqueueNDRangeKernel(s.kernel_processPixelStage2,
				   cl::NullRange, cl::NDRange(IMAGE_SIZE), cl::NullRange,
				   &s.eventPPS1, &s.eventPPS2[0]);

#if defined(HAVE_GLEW)
	if (m_type & K4W2_DECODER_ENABLE_OPENGL)
	    queue.enqueueReleaseGLObjects(&objs);
#endif
    }
    catch (const cl::Error & err) {
	VERBOSE("%s %x", err.what(), err.err() );
	return false;
    }
    return true;
}

bool
DecoderCL::get_gl_texture(int slot, unsigned int option, unsigned int *texture)
{
#if defined(HAVE_GLEW)
    int idx = (0==option)?0:1;
    Slot& s = m_slot[slot];
    *texture = s.texture.name[idx];
    return true;
#else
    return false;
#endif
}

static cl::size_t<3> mk_cl_size3(int x, int y, int z)
{
    cl::size_t<3> tmp;
    tmp[0] = x;
    tmp[1] = y;
    tmp[2] = z;
    return tmp;
}

bool
DecoderCL::fetch(int slot, void *dst, int dst_length)
{
    Slot& s = m_slot[slot];
    try
    {
	static const cl::size_t<3> origin = mk_cl_size3(0, 0, 0);
	static const cl::size_t<3> region = mk_cl_size3(512, 424, 1);
	queue.enqueueReadImage(s.image[1], CL_FALSE,
			       origin, region,
			       0,0,
			       (char*)dst + buf_depth_size,
			       &s.eventPPS1, &s.event0);
	queue.enqueueReadImage(s.image[0], CL_FALSE,
			       origin, region,
			       0,0,
			       dst,
			       &s.eventPPS2, &s.event1);
	s.event0.wait();
	s.event1.wait();
    }
    catch (const cl::Error & err) {
	VERBOSE("%s %x", err.what(), err.err() );
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
depth_cl_open(k4w2_decoder_t ctx, const unsigned int type)
{
    if ( (type & K4W2_DECODER_TYPE_MASK) != K4W2_DECODER_DEPTH) {
	return K4W2_ERROR;
    }
    if ( type & K4W2_DECODER_DISABLE_OPENCL ) {
	VERBOSE("K4W2_DECODER_DISABLE_OPENCL is set");
	return K4W2_ERROR;
    }

    depth_cl * d = (depth_cl *)ctx;

    // Call placement new to initialize OpenCL's variables
    // because it was allocated by malloc()
    new (&d->dcl) DecoderCL;

    parameters params;
    init_params(&params);

    if (! d->dcl.setup(params, ctx->num_slot, type) ) {
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
depth_cl_get_gl_texture(k4w2_decoder_t ctx, int slot, unsigned int option, unsigned int *texturename)
{
    depth_cl * d = (depth_cl *)ctx;
    bool r = d->dcl.get_gl_texture(slot, option, texturename);
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
#if defined(HAVE_GLEW)
    ops.get_gl_texture = depth_cl_get_gl_texture;
#endif
    ops.fetch	= depth_cl_fetch;
    ops.close	= depth_cl_close;
    
    k4w2_register_decoder("depth OpenCL", &ops, sizeof(depth_cl));
}

END_EXTERN_C

// Local Variables:
// mode: c++
// c-basic-offset:  4
// End:
