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

#include <string.h> /* for strstr() */
#include <assert.h>

#define _USE_MATH_DEFINES
#include <math.h>
#if ! defined M_PI
#  define M_PI 3.14159265358979323846264338327
#endif

/* This module uses OpenCL 1.2 because NVIDIA's driver doesn't support OpenCL 2.0 yet. */
#define CL_USE_DEPRECATED_OPENCL_1_2_APIS
#if defined(__APPLE__ ) || defined(__MACOSX)
#  include <OpenCL/cl.h>
#  include <OpenCL/cl_gl.h>
#else
#  include <CL/cl.h>
#  include <CL/cl_gl.h>
#endif

#define USE_OPENCL_ENUM
#if defined USE_OPENCL_ENUM
#  include "opencl_enum.c"
#else
/* this function is not thread safe */
static char * opencl_strenum(cl_int e) {
    char tmp[1024];
    snprintf(tmp, sizeof(tmp), "%x", e);
    return tmp;
}
#endif

#include "module.h"
#include "libk4w2/kinect2.h"

#define CHECK_GL() do {					\
	GLenum e;					\
	while ( (e = glGetError()) != GL_NO_ERROR ) {	\
	    VERBOSE("glGetError() returns '%x'", e);	\
	}						\
    } while(0)

#define CHK_CL(exp) do {						\
	cl_int e =  exp;						\
	if (CL_SUCCESS != e) {						\
	    VERBOSE(# exp " returns '%s'",  opencl_strenum(e) );	\
	}								\
    } while (0)

#define IMAGE_SIZE (512 * 424)

#define buf_packet_size KINECT2_DEPTH_FRAME_SIZE * 10
#define buf_a_size	IMAGE_SIZE * sizeof(cl_float3)
#define buf_b_size	IMAGE_SIZE * sizeof(cl_float3)
#define buf_n_size	IMAGE_SIZE * sizeof(cl_float3)
#define buf_ir_size	IMAGE_SIZE * sizeof(cl_float)
#define buf_depth_size  IMAGE_SIZE * sizeof(cl_float)

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

typedef struct Slot_tag Slot;
struct Slot_tag {
    cl_kernel kernel_1;
    cl_kernel kernel_2;

    cl_mem buf_packet;

    cl_mem buf_a;
    cl_mem buf_b;
    cl_mem buf_n;

#if defined(HAVE_GLEW)
    union {
	struct {
	    GLuint depth;
	    GLuint ir;
	};
	GLuint name[2];
    } texture;
#endif

    cl_mem image[2]; /* 0:depth, 1:ir */
    cl_event eventWrite[2];
    cl_event eventPPS1[1];
    cl_event eventPPS2[1];
    cl_event event0, event1;
};


/**
 * @class DecoderCL depth_cl.c
 *
 */
typedef struct DecoderCL_tag DecoderCL;
struct DecoderCL_tag
{
    cl_context context;
    cl_command_queue queue;
    cl_program program;

    /* Read only buffers */
    cl_mem buf_lut11to16;
    cl_mem buf_p0_table;
    cl_mem buf_x_table;
    cl_mem buf_z_table;

    Slot *m_slot;
    size_t m_num_slot;

    unsigned int m_type;
};

static void open_slot(Slot *s, const DecoderCL *decoder);

static int open_decoder(DecoderCL *decoder, const struct parameters *params, size_t num_slot, unsigned int type);
static int set_params(DecoderCL *decoder,
		      const struct kinect2_color_camera_param * color,
		      const struct kinect2_depth_camera_param * depth,
		      const struct kinect2_p0table * p0table);

static int request(DecoderCL *decoder,
		   int slot, const void *ptr, int length);
static int fetch(DecoderCL *decoder,
		 int slot, void *dst, int dst_length);
static int get_gl_texture(DecoderCL *decoder,
			  int slot, unsigned int option, unsigned int *texture);


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
fill_trig_table(const struct kinect2_p0table *src, cl_float3 *dst)
{
    int r;
    for(r = 0; r < 424; ++r)
    {
	cl_float3 *it = &dst[r * 512];
	const uint16_t *it0 = &src->p0table0[r * 512];
	const uint16_t *it1 = &src->p0table1[r * 512];
	const uint16_t *it2 = &src->p0table2[r * 512];
	int c;
	for(c = 0; c < 512; ++c, ++it, ++it0, ++it1, ++it2)
	{
	    it->s[0] = -((float) * it0) * 0.000031 * M_PI;
	    it->s[1] = -((float) * it1) * 0.000031 * M_PI;
	    it->s[2] = -((float) * it2) * 0.000031 * M_PI;
	    //it->s[3] = 0.0f;
	}
    }
}

static char *
generateOptions(const struct parameters *params) 
{
    const size_t size = 2 * 1024;
    char *buf = (char *)malloc(size);
    char *p = buf;
    char *tail = buf + size -1;

#define LEFT(x) ( ((x)>0)?(x):0 )
#define FMT "%.16ef"

    p += snprintf(p, LEFT(tail - p), " -D KINECT2_DEPTH_FRAME_SIZE=%zd", KINECT2_DEPTH_FRAME_SIZE);
    p += snprintf(p, LEFT(tail - p), " -D BFI_BITMASK=0x180");

    p += snprintf(p, LEFT(tail - p), " -D AB_MULTIPLIER=" FMT, params->ab_multiplier);
    p += snprintf(p, LEFT(tail - p), " -D AB_MULTIPLIER_PER_FRQ0=" FMT, params->ab_multiplier_per_frq[0]);
    p += snprintf(p, LEFT(tail - p), " -D AB_MULTIPLIER_PER_FRQ1=" FMT, params->ab_multiplier_per_frq[1]);
    p += snprintf(p, LEFT(tail - p), " -D AB_MULTIPLIER_PER_FRQ2=" FMT, params->ab_multiplier_per_frq[2]);
    p += snprintf(p, LEFT(tail - p), " -D AB_OUTPUT_MULTIPLIER=" FMT, params->ab_output_multiplier);

    p += snprintf(p, LEFT(tail - p), " -D PHASE_IN_RAD0=" FMT, params->phase_in_rad[0]);
    p += snprintf(p, LEFT(tail - p), " -D PHASE_IN_RAD1=" FMT, params->phase_in_rad[1]);
    p += snprintf(p, LEFT(tail - p), " -D PHASE_IN_RAD2=" FMT, params->phase_in_rad[2]);

    p += snprintf(p, LEFT(tail - p), " -D PHASE_OFFSET=" FMT, params->phase_offset);
    p += snprintf(p, LEFT(tail - p), " -D UNAMBIGIOUS_DIST=" FMT, params->unambigious_dist);
    p += snprintf(p, LEFT(tail - p), " -D INDIVIDUAL_AB_THRESHOLD=" FMT, params->individual_ab_threshold);
    p += snprintf(p, LEFT(tail - p), " -D AB_THRESHOLD=" FMT, params->ab_threshold);
    p += snprintf(p, LEFT(tail - p), " -D AB_CONFIDENCE_SLOPE=" FMT, params->ab_confidence_slope);
    p += snprintf(p, LEFT(tail - p), " -D AB_CONFIDENCE_OFFSET=" FMT, params->ab_confidence_offset);
    p += snprintf(p, LEFT(tail - p), " -D MIN_DEALIAS_CONFIDENCE=" FMT, params->min_dealias_confidence);
    p += snprintf(p, LEFT(tail - p), " -D MAX_DEALIAS_CONFIDENCE=" FMT, params->max_dealias_confidence);

#undef LEFT
#undef FMT

    return buf;
}


static void
open_slot(Slot *s, const DecoderCL *decoder)
{
    cl_int err;
    s->buf_packet = clCreateBuffer(decoder->context, CL_READ_ONLY_CACHE,  buf_packet_size, NULL, &err);
    s->buf_a      = clCreateBuffer(decoder->context, CL_READ_WRITE_CACHE, buf_a_size, NULL, &err);
    s->buf_b      = clCreateBuffer(decoder->context, CL_READ_WRITE_CACHE, buf_b_size, NULL, &err);
    s->buf_n      = clCreateBuffer(decoder->context, CL_READ_WRITE_CACHE, buf_n_size, NULL, &err);

    cl_image_format format;
    format.image_channel_order = CL_R;
    format.image_channel_data_type = CL_FLOAT;

    cl_image_desc desc;
    desc.image_type = CL_MEM_OBJECT_IMAGE2D;
    desc.image_width = 512;
    desc.image_height = 424;
    desc.image_depth = 0;
    desc.image_array_size = 0;
    desc.image_row_pitch = 0;
    desc.image_slice_pitch = 0;
    desc.num_mip_levels = 0;
    desc.num_samples = 0;
    desc.buffer = NULL;

#if defined(HAVE_GLEW)
    if (decoder->m_type & K4W2_DECODER_ENABLE_OPENGL) {
	glGenTextures(2, &s->texture.name[0]);
	size_t i;
	for (i = 0; i < 2; ++i) {
	    glBindTexture(GL_TEXTURE_2D, s->texture.name[i]);
	    glTexStorage2D(GL_TEXTURE_2D,
			   1,
			   GL_R32F,
			   512, 424);
	    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	    CHECK_GL();

	    s->image[i] = clCreateFromGLTexture(decoder->context, CL_MEM_WRITE_ONLY, GL_TEXTURE_2D, 0,
						s->texture.name[i], &err);
	    if (CL_SUCCESS != err) {
		ABORT("failed, err: %d", err);
	    }
	}
    } else {
	size_t i;
	for (i = 0; i < 2; ++i) {
	    s->texture.name[i] = 0;
	    s->image[i] = clCreateImage(decoder->context, CL_MEM_WRITE_ONLY,
					&format,
					&desc,
					NULL, &err);
	}
    }
#else
    for (size_t i = 0; i < 2; ++i) {
	s->image[i] = clCreateImage(decoder->context, CL_MEM_WRITE_ONLY,
				    &format,
				    &desc,
				    NULL, &err);
    }
#endif

    s->kernel_1 = clCreateKernel(decoder->program, "processPixelStage1", &err);
    if (!s->kernel_1)
	ABORT("no processPixelStage1");
    CHK_CL( clSetKernelArg(s->kernel_1, 0, sizeof(cl_mem), &decoder->buf_lut11to16) );
    CHK_CL( clSetKernelArg(s->kernel_1, 1, sizeof(cl_mem), &decoder->buf_z_table) );
    CHK_CL( clSetKernelArg(s->kernel_1, 2, sizeof(cl_mem), &decoder->buf_p0_table) );
    CHK_CL( clSetKernelArg(s->kernel_1, 3, sizeof(cl_mem), &s->buf_packet) );
    CHK_CL( clSetKernelArg(s->kernel_1, 4, sizeof(cl_mem), &s->buf_a) );
    CHK_CL( clSetKernelArg(s->kernel_1, 5, sizeof(cl_mem), &s->buf_b) );
    CHK_CL( clSetKernelArg(s->kernel_1, 6, sizeof(cl_mem), &s->buf_n) );
    CHK_CL( clSetKernelArg(s->kernel_1, 7, sizeof(cl_mem), &s->image[1]) );

    s->kernel_2 = clCreateKernel(decoder->program, "processPixelStage2", &err);
    if (!s->kernel_2)
	ABORT("no processPixelStage2");
    CHK_CL( clSetKernelArg(s->kernel_2, 0, sizeof(cl_mem), &s->buf_a) );
    CHK_CL( clSetKernelArg(s->kernel_2, 1, sizeof(cl_mem), &s->buf_b) );
    CHK_CL( clSetKernelArg(s->kernel_2, 2, sizeof(cl_mem), &decoder->buf_x_table) );
    CHK_CL( clSetKernelArg(s->kernel_2, 3, sizeof(cl_mem), &decoder->buf_z_table) );
    CHK_CL( clSetKernelArg(s->kernel_2, 4, sizeof(cl_mem), &s->image[0]) );
}

static void
close_slot(Slot *s, unsigned int type)
{
    CHK_CL( clReleaseMemObject(s->buf_packet) );
    CHK_CL( clReleaseMemObject(s->buf_a) );
    CHK_CL( clReleaseMemObject(s->buf_b) );
    CHK_CL( clReleaseMemObject(s->buf_n) );
    size_t i;
    for (i = 0; i < 2; ++i) {
	CHK_CL( clReleaseMemObject(s->image[i]) );
    }
    if (type & K4W2_DECODER_ENABLE_OPENGL) {
	glDeleteTextures(2, &s->texture.name[0]);
    }

    CHK_CL( clReleaseKernel(s->kernel_1) );
    CHK_CL( clReleaseKernel(s->kernel_2) );
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
static int check_opengl_context(cl_device_id device_id)
{

#if defined(__APPLE__) || defined(MACOSX)
    if (!get_current_cgl_share_group()) {
	VERBOSE("No CGLContext attached.");
	return 0;
    }
#elif defined(WIN32)
    if (!(wglGetCurrentContext() && wglGetCurrentDC())) {
	VERBOSE("No OpenGL context attached");
	return 0;
    }
#else
    if (!(glXGetCurrentContext() && glXGetCurrentDisplay())) {
	VERBOSE("No GLX context attached.");
	return 0;
    }
#endif

#if defined(__APPLE__) || defined(MACOSX)
    static const char * CL_GL_SHARING_EXT = "cl_APPLE_gl_sharing";
#else
    static const char * CL_GL_SHARING_EXT = "cl_khr_gl_sharing";
#endif

    int found = 0;
    size_t ext_size = 2*1024;
    char *ext_buf = (char *)malloc(ext_size);
    cl_int err;

    err = clGetDeviceInfo(device_id, CL_DEVICE_EXTENSIONS,
			  ext_size, ext_buf, &ext_size);
    if (err != CL_SUCCESS) {
	VERBOSE("clGetDeviceInfo() failed");
    } else if (NULL == strstr(&ext_buf[0], CL_GL_SHARING_EXT)) {
	VERBOSE("%s is not supported", CL_GL_SHARING_EXT);
    } else {
	found = 1;
    }
    free(ext_buf);
    return found;
}

#endif /* #if defined(HAVE_GLEW) */


static void
setup_cl_context_properties(cl_context_properties prop[], size_t max_size,
			    cl_platform_id platform_id,
			    unsigned int type)
{
    if (type & K4W2_DECODER_ENABLE_OPENGL) {
	cl_context_properties tmp[] = {
	    CL_CONTEXT_PLATFORM, (cl_context_properties)platform_id,
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
	    CL_CONTEXT_PLATFORM, (cl_context_properties)platform_id,
	    0
	};
	memcpy(prop, tmp, sizeof(tmp));
    }
}

static int
open_decoder(DecoderCL *decoder,
	      const struct parameters *params,
	      size_t num_slot,
	      const unsigned int type)
{
    decoder->m_type = type;
    cl_int err = CL_SUCCESS;

    cl_platform_id platforms[10] = {0};
    cl_uint num_platforms = 0;
    CHK_CL( clGetPlatformIDs(ARRAY_SIZE(platforms),
			     platforms,
			     &num_platforms) );
    if (0 == num_platforms) {
	ABORT("no opencl platform found");
    }

    cl_context_properties properties[10] = {0};
    setup_cl_context_properties(properties, sizeof(properties),
				platforms[0],
				type);

    cl_device_id devices[10] = {0};
    cl_uint num_devices = 0;
    CHK_CL(clGetDeviceIDs(platforms[0],
			  CL_DEVICE_TYPE_GPU,
			  ARRAY_SIZE(devices),
			  devices,
			  &num_devices) );
    if (0 == num_devices) {
	ABORT("no opencl device found");
    }

    decoder->context = clCreateContext(properties,
				       num_devices,
				       &devices[0],
				       NULL,
				       NULL,
				       &err);

    if (CL_SUCCESS != err) {
	ABORT("create context failed");
    }

    decoder->queue   = clCreateCommandQueue(decoder->context,
					    devices[0],
					    0,
					    &err);

    static const char *searchpath[] = {
	K4W2_SRCDIR"/decoder_cl",
	K4W2_DATADIR,
    };

    const int MAX_SOURCECODE_SIZE = 20 * 1024;
    char* sourcecode = (char *)malloc(MAX_SOURCECODE_SIZE);
    size_t sourcelength;
    int r = k4w2_search_and_load(searchpath, ARRAY_SIZE(searchpath),
				 "depth.cl",
				 sourcecode, MAX_SOURCECODE_SIZE,
				 &sourcelength);
    if (K4W2_SUCCESS != r) {
	ABORT("failed to load ");
    }
    sourcecode[sourcelength] = '\0';

#if defined(HAVE_GLEW)
    if ( decoder->m_type & K4W2_DECODER_ENABLE_OPENGL ) {
	check_opengl_context(devices[0]);
    }

#endif


    const char *src[] = {sourcecode};
    const size_t len[] = {sourcelength};
    decoder->program = clCreateProgramWithSource(decoder->context,
						 1,
						 &src[0],
						 &len[0],
						 &err);
    if (!decoder->program || err != CL_SUCCESS) {
	ABORT("create program failed. %s", opencl_strenum(err));
    }

    const char *options = generateOptions(params);
    err = clBuildProgram(decoder->program,
			 num_devices,
			 &devices[0],
			 options,
			 NULL,
			 NULL);
    if (CL_SUCCESS != err) {
	if (err == CL_BUILD_PROGRAM_FAILURE) {
	    cl_build_status status;
	    CHK_CL( clGetProgramBuildInfo(decoder->program, devices[0],
					  CL_PROGRAM_BUILD_STATUS,
					  sizeof(cl_build_status), &status, NULL) );
	    VERBOSE("build status: %s", opencl_strenum(status));
	    
	    size_t buflen = 16 * 1024;
	    char *buf = (char *)malloc(buflen);
	    CHK_CL( clGetProgramBuildInfo(decoder->program, devices[0],
					  CL_PROGRAM_BUILD_LOG,
					  buflen, buf, &buflen) );
	    buf[ buflen ] = '\0';
	    ABORT("build log: %s", buf);
	} else {
	    ABORT("clBuildProgram() returns %s", opencl_strenum(err) );
	}
    }

    decoder->buf_lut11to16 = clCreateBuffer(decoder->context, CL_READ_ONLY_CACHE,
					    2*2048UL, NULL,
					    &err);
    decoder->buf_p0_table  = clCreateBuffer(decoder->context, CL_READ_ONLY_CACHE,
					    IMAGE_SIZE * sizeof(cl_float3), NULL,
					    &err);
    decoder->buf_x_table = clCreateBuffer(decoder->context, CL_READ_ONLY_CACHE,
					  IMAGE_SIZE * sizeof(cl_float3), NULL,
					  &err);
    decoder->buf_z_table = clCreateBuffer(decoder->context, CL_READ_ONLY_CACHE,
					  IMAGE_SIZE * sizeof(cl_float3), NULL,
					  &err);

    decoder->m_slot = (Slot *)malloc(sizeof(Slot) * num_slot);
    decoder->m_num_slot = num_slot;
    int i;
    for (i=0; i<num_slot; ++i) {
	open_slot(&decoder->m_slot[i], decoder);
    }

    free(sourcecode);

    return K4W2_SUCCESS;
}

static void
close_decoder(DecoderCL *decoder)
{
    if (decoder)
	return ;

    int i;
    for (i=0; i<decoder->m_num_slot; ++i) {
	close_slot(&decoder->m_slot[i], decoder->m_type);
    }
    free( decoder->m_slot );

    CHK_CL( clReleaseMemObject(decoder->buf_lut11to16) );
    CHK_CL( clReleaseMemObject(decoder->buf_p0_table) );
    CHK_CL( clReleaseMemObject(decoder->buf_x_table) );
    CHK_CL( clReleaseMemObject(decoder->buf_z_table) );

    CHK_CL( clReleaseProgram(decoder->program) );
    CHK_CL( clReleaseCommandQueue(decoder->queue) );

    CHK_CL( clReleaseContext(decoder->context) );
}

static int
set_params(DecoderCL *decoder,
	   const struct kinect2_color_camera_param * color,
	   const struct kinect2_depth_camera_param * depth,
	   const struct kinect2_p0table * p0table)
{
    {
	short lut[2048];
	int r = k4w2_create_lut_table(lut, sizeof(lut));
	if (K4W2_SUCCESS != r)
	    return r;
	CHK_CL( clEnqueueWriteBuffer(decoder->queue,
				     decoder->buf_lut11to16, CL_TRUE,
				     0, sizeof(lut), lut,
				     0, NULL, NULL) );
    }

    {
	cl_float3 tmp[IMAGE_SIZE];
	fill_trig_table(p0table, tmp);
	CHK_CL( clEnqueueWriteBuffer(decoder->queue,
				     decoder->buf_p0_table, CL_TRUE,
				     0, sizeof(tmp), tmp,
				     0, NULL, NULL) );
    }

    {
	float x_table[IMAGE_SIZE];
	float z_table[IMAGE_SIZE];
	int r = k4w2_create_xz_table(depth,
				     x_table, sizeof(x_table),
				     z_table, sizeof(z_table));
	if (K4W2_SUCCESS != r)
	    return r;
	CHK_CL (clEnqueueWriteBuffer(decoder->queue,
				     decoder->buf_x_table, CL_TRUE,
				     0, sizeof(x_table), x_table,
				     0, NULL, NULL) );
	CHK_CL (clEnqueueWriteBuffer(decoder->queue,
				     decoder->buf_z_table, CL_TRUE,
				     0, sizeof(z_table), z_table,
				     0, NULL, NULL) );
    }


    CHK_CL( clFinish(decoder->queue) );
    
    return K4W2_SUCCESS;
}


static int
request(DecoderCL *decoder, int slot, const void *ptr, int length)
{
    if (KINECT2_DEPTH_FRAME_SIZE*10 != length) {
	return K4W2_ERROR;
    }

    Slot* s = &decoder->m_slot[slot];

    cl_mem objs[2] = {s->image[0], s->image[1]};

    CHK_CL( clEnqueueWriteBuffer(decoder->queue,
				 s->buf_packet, CL_FALSE, 0, length, ptr,
				 0, NULL,
				 &s->eventWrite[0]) );

    // !!FIXME!!
    int num_event_write = 1;
#if defined(HAVE_GLEW)
    if (decoder->m_type & K4W2_DECODER_ENABLE_OPENGL) {
	CHK_CL( clEnqueueAcquireGLObjects(decoder->queue,
					  ARRAY_SIZE(objs), &objs[0],
					  0, NULL,
					  &s->eventWrite[1]) );
	num_event_write = 2;
    }
#endif

    static const size_t global_work_size[1] = {IMAGE_SIZE}; 
    CHK_CL( clEnqueueNDRangeKernel(decoder->queue,
				   s->kernel_1,
				   1,
				   NULL,
				   global_work_size,
				   NULL,
				   num_event_write, &s->eventWrite[0],
				   &s->eventPPS1[0]) );

    CHK_CL( clEnqueueNDRangeKernel(decoder->queue,
				   s->kernel_2,
				   1,
				   NULL,
				   global_work_size,
				   NULL,
				   1, &s->eventPPS1[0],
				   &s->eventPPS2[0]) );

#if defined(HAVE_GLEW)
    if (decoder->m_type & K4W2_DECODER_ENABLE_OPENGL) {
	CHK_CL( clEnqueueReleaseGLObjects(decoder->queue,
					  ARRAY_SIZE(objs), &objs[0],
					  1, &s->eventPPS2[0],
					  NULL) );
    }
#endif

    return K4W2_SUCCESS;
}

static int
get_gl_texture(DecoderCL *decoder, int slot, unsigned int option, unsigned int *texture)
{
#if defined(HAVE_GLEW)
    int idx = (0==option)?0:1;
    Slot* s = &decoder->m_slot[slot];
    *texture = s->texture.name[idx];
    return K4W2_SUCCESS;
#else
    return K4W2_ERROR;
#endif
}

static int
fetch(DecoderCL *decoder, int slot, void *dst, int dst_length)
{
    Slot* s = &decoder->m_slot[slot];
    static const size_t origin[3] = {0,0,0};
    static const size_t region[3] = {512, 424, 1};

    CHK_CL( clEnqueueReadImage(decoder->queue,
			       s->image[1],
			       CL_FALSE, 
			       origin, region,
			       0,0,
			       (char*)dst + buf_depth_size,
			       ARRAY_SIZE(s->eventPPS1), &s->eventPPS1[0],
			       &s->event0) );

    CHK_CL( clEnqueueReadImage(decoder->queue,
			       s->image[0],
			       CL_FALSE, 
			       origin, region,
			       0,0,
			       dst,
			       ARRAY_SIZE(s->eventPPS2), &s->eventPPS2[0],
			       &s->event1) );

    CHK_CL( clWaitForEvents(1, &s->event0) );
    CHK_CL( clWaitForEvents(1, &s->event1) );

    return K4W2_SUCCESS;
}

typedef struct {
    struct k4w2_decoder_ctx decoder; 
    DecoderCL dcl;
} depth_cl;

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

    struct parameters params;
    init_params(&params);

    return open_decoder(&d->dcl, &params, ctx->num_slot, type);
}

static int
depth_cl_set_params(k4w2_decoder_t ctx, 
		     struct kinect2_color_camera_param * color,
		     struct kinect2_depth_camera_param * depth,
		     struct kinect2_p0table * p0table)
{
    depth_cl * d = (depth_cl *)ctx;
    return set_params(&d->dcl, color, depth, p0table);
}

static int
depth_cl_request(k4w2_decoder_t ctx, int slot, const void *src, int src_length)
{
    depth_cl * d = (depth_cl *)ctx;
    return request(&d->dcl, slot, src, src_length);
}

static int
depth_cl_fetch(k4w2_decoder_t ctx, int slot, void *dst, int dst_length)
{
    depth_cl * d = (depth_cl *)ctx;
    return fetch(&d->dcl, slot, dst, dst_length);
}

static int
depth_cl_get_gl_texture(k4w2_decoder_t ctx, int slot, unsigned int option, unsigned int *texturename)
{
    depth_cl * d = (depth_cl *)ctx;
    return get_gl_texture(&d->dcl, slot, option, texturename);
}

static int
depth_cl_close(k4w2_decoder_t ctx)
{
    depth_cl * d = (depth_cl *)ctx;

    close_decoder(&d->dcl);

    return K4W2_SUCCESS;
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

// Local Variables:
// mode: c
// c-basic-offset:  4
// End:
