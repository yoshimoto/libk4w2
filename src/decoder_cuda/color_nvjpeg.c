/**
 * @file   color_nvjpeg.c
 * @author Hiromasa YOSHIMOTO
 * @date   Wed Aug 15 17:01:23 2018
 *
 * @brief  
 */
#if defined(HAVE_GLEW)
#include <GL/glew.h>
#endif

#include "module.h"

#if ! defined HAVE_NVJPEG
#  error "nvJPEG is not installed"
#else

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include "nvjpeg.h"

#include <assert.h>

typedef struct {
    GLuint texture_id;
    nvjpegJpegState_t jpeg;

    nvjpegImage_t image;
    cudaGraphicsResource_t gres;
    GLuint bufobj;

    int phase;
} decoder_slot;

typedef struct {
    struct k4w2_decoder_ctx decoder; 

    nvjpegHandle_t handle;
    cudaStream_t   stream;
    decoder_slot *slot;

    nvjpegOutputFormat_t outputfmt;
    // Image format of slot->bufobj.
    GLenum buffmt;
    int nChannels;
} decoder_nvjpeg;

static const char *
nvjpeg_strerro(nvjpegStatus_t s)
{
    static const char *msg[] = {
	"Success",
	"Not initialized",
	"Invalid parameter",
	"Bad jpeg",
	"Jpeg not supported",
	"Allocator failure",
	"Execution failed",
	"Arch mismatch",
	"Internal error",
    };
    if (0<=s && s < sizeof(msg)/sizeof(msg[0]))
	return msg[s];
    else
	return "Unknown error";
}

#define CUDA_CHECK_ERR() do {					\
	cudaError_t e = cudaGetLastError();			\
	if(e!=cudaSuccess) {					\
	    VERBOSE("Cuda failure: %s", cudaGetErrorString(e));	\
	}							\
    } while(0)


static int
color_nvjpeg_open(k4w2_decoder_t ctx, unsigned int type)
{
    size_t s;
    decoder_nvjpeg * d = (decoder_nvjpeg *)ctx;

    if ( (type & K4W2_DECODER_TYPE_MASK) != K4W2_DECODER_COLOR)
	goto err;
    if ( type & K4W2_DECODER_DISABLE_CUDA )
	goto err;

    nvjpegStatus_t res;
    res = nvjpegCreate(NVJPEG_BACKEND_DEFAULT, NULL, &d->handle);
    if (res) {
	VERBOSE("nvjpegCreate() failed; %s", nvjpeg_strerro(res));
	goto err;
    }

    cudaStreamCreateWithFlags(&d->stream, cudaStreamNonBlocking);
    CUDA_CHECK_ERR();

    int use_y_component_only = 0;

    GLenum internalfmt;
    if (use_y_component_only) {
	internalfmt = GL_R8;
	d->outputfmt = NVJPEG_OUTPUT_Y;
	d->nChannels = 1;
	d->buffmt = GL_RED;
    } else {
	internalfmt = GL_RGB8;
	d->outputfmt = NVJPEG_OUTPUT_RGBI;
	d->nChannels = 3;
	d->buffmt = GL_RGB;
    }
    assert(1 <= ctx->num_slot);
    d->slot = (decoder_slot*) malloc (sizeof(decoder_slot) * ctx->num_slot);
    memset(d->slot, 0, sizeof(decoder_slot) * ctx->num_slot);
    for (s = 0; s < ctx->num_slot; ++s) {
	decoder_slot *slot = &d->slot[s];
	res = nvjpegJpegStateCreate(d->handle, &slot->jpeg);
	if (res) {
	    VERBOSE("nvjpegJpegStateCreate() failed; %s", nvjpeg_strerro(res));
	    goto err;
	}
    }

    if (type & K4W2_DECODER_ENABLE_OPENGL) {
	// Allocate slot->image from OpenGL's buffer object
	for (s = 0; s < ctx->num_slot; ++s) {
	    decoder_slot *slot = &d->slot[s];
	    glCreateTextures(GL_TEXTURE_2D, 1, &slot->texture_id);
	    glTextureStorage2D(slot->texture_id,
			       1, internalfmt, 1920, 1080);

	    glCreateBuffers(1, &slot->bufobj);
	    glNamedBufferData(slot->bufobj, 1920*1080*d->nChannels,
			      NULL, GL_STREAM_COPY);

	    // Prepares for opengl interoperation
	    cudaGraphicsGLRegisterBuffer(&slot->gres, slot->bufobj,
					 cudaGraphicsMapFlagsNone);

	    slot->image.channel[0] = NULL;
	    slot->image.pitch[0] = 1920 * d->nChannels;

	    CUDA_CHECK_ERR();
	}
    } else {
	// Allocate slot->image from cuda's device memory
	for (s = 0; s < ctx->num_slot; ++s) {
	    decoder_slot *slot = &d->slot[s];
	    slot->bufobj = 0;

	    cudaMalloc((void**)&slot->image.channel[0], 1920 * 1080 * d->nChannels);
	    CUDA_CHECK_ERR();
	    slot->image.pitch[0] = 1920 * d->nChannels;
	}
    }

    return K4W2_SUCCESS;
err:
    return K4W2_ERROR;
}

static void
decode_phase3_if_needed(decoder_nvjpeg *d, decoder_slot *slot)
{
    if (3 != slot->phase) {
	nvjpegDecodePhaseThree(d->handle, slot->jpeg,
			       &slot->image, d->stream);
	cudaStreamSynchronize(d->stream);
	CUDA_CHECK_ERR();
	slot->phase = 3;
    }
}

static int
color_nvjpeg_request(k4w2_decoder_t ctx, int slotNo, const void *src, int src_length)
{
    decoder_nvjpeg * d = (decoder_nvjpeg *)ctx;
    struct kinect2_color_header* h = (struct kinect2_color_header*)src;
    decoder_slot *slot = &d->slot[slotNo % ctx->num_slot];

    CUDA_CHECK_ERR();

    if (slot->bufobj) {
	// Decode jpeg into OpenGL's buffer object
	cudaGraphicsMapResources(1, &slot->gres, d->stream);
	CUDA_CHECK_ERR();
	cudaGraphicsResourceGetMappedPointer((void**)&slot->image.channel[0],
					     NULL,
					     slot->gres);
	CUDA_CHECK_ERR();
    } else {
	// Decode jpeg into CUDA's device memory, which
	// is allocated already in color_nvjpeg_open().
    }

    nvjpegStatus_t res;
    res = nvjpegDecodePhaseOne(d->handle, slot->jpeg,
			       h->image, src_length, d->outputfmt,
			       d->stream);
    if (res) {
	VERBOSE("nvjpegDecodePhaseOne() failed; %s", nvjpeg_strerro(res));
    }
    res = nvjpegDecodePhaseTwo(d->handle, slot->jpeg, d->stream);
    if (res) {
	VERBOSE("nvjpegDecodePhaseTwo() failed; %s", nvjpeg_strerro(res));
    }
    slot->phase = 2;


    if (slot->bufobj) {
	// Updated OpenGL's texture
	//
	//
	decode_phase3_if_needed(d, slot);
	cudaGraphicsUnmapResources(1, &slot->gres, d->stream);
	CUDA_CHECK_ERR();

	// Update opengl's texture
	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, slot->bufobj);
	glTextureSubImage2D(slot->texture_id,
			    0,
			    0,0,
			    1920, 1080,
			    d->buffmt, GL_UNSIGNED_BYTE, NULL);
	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
    }

    return K4W2_SUCCESS;
}



static int
color_nvjpeg_fetch(k4w2_decoder_t ctx, int slotNo, void *dst, int dst_length)
{
    decoder_nvjpeg * d = (decoder_nvjpeg *)ctx;
    decoder_slot *slot = &d->slot[slotNo % ctx->num_slot];

    if (slot->bufobj) {
	// Decoded image is stored in OpenGL's buffer object.
	glGetNamedBufferSubData(slot->bufobj,
				0,
				1920*1080*d->nChannels,
				dst);
    } else {
	decode_phase3_if_needed(d, slot);
	// Decoded image is stored in CUDA's device memory.
	cudaMemcpy(dst, slot->image.channel[0], dst_length, cudaMemcpyDeviceToHost);
	CUDA_CHECK_ERR();
    }
    return K4W2_SUCCESS;
}

static int
color_nvjpeg_get_gl_texture(k4w2_decoder_t ctx, int slot_No, unsigned int options, unsigned int *texturename)
{
    decoder_nvjpeg * d = (decoder_nvjpeg *)ctx;
    decoder_slot *slot = &d->slot[slot_No % ctx->num_slot];
    *texturename = slot->texture_id;

    return K4W2_SUCCESS;
}

static int
color_nvjpeg_get_colorspace(k4w2_decoder_t ctx)
{
    return K4W2_COLORSPACE_RGB;
}

static int
color_nvjpeg_close(k4w2_decoder_t ctx)
{
    decoder_nvjpeg * d = (decoder_nvjpeg *)ctx;

    if (d) {
	cudaStreamDestroy(d->stream);
	CUDA_CHECK_ERR();
    }
    if (d && d->slot) {
	size_t s;
	for (s = 0; s < ctx->num_slot; ++s) {
	    decoder_slot *slot = &d->slot[s];

	    if (slot->bufobj) {
		if (slot->gres) {
		    cudaGraphicsUnregisterResource(slot->gres);
		    CUDA_CHECK_ERR();
		}
		glDeleteBuffers(1, &slot->bufobj);
		glDeleteTextures(1, &slot->texture_id);
	    } else {
		if (slot->image.channel[0])
		    cudaFree(slot->image.channel[0]);
	    }
	    if (slot->jpeg) {
		nvjpegJpegStateDestroy(slot->jpeg);
		slot->jpeg=0;
	    }
	}
	free(d->slot);
	d->slot = NULL;
    }
    if (d && d->handle) {
	nvjpegDestroy(d->handle);
	d->handle = NULL;
    }
    return K4W2_SUCCESS;
}

static const k4w2_decoder_ops ops = {
    .open	= color_nvjpeg_open,
    .set_params = NULL,
    .get_colorspace = color_nvjpeg_get_colorspace,
    .request	= color_nvjpeg_request,
    .get_gl_texture = color_nvjpeg_get_gl_texture,
    .fetch	= color_nvjpeg_fetch,
    .close	= color_nvjpeg_close,
};

REGISTER_MODULE(k4w2_decoder_color_nvjpeg_init)
{
    k4w2_register_decoder("color nvjpeg", &ops, sizeof(decoder_nvjpeg));
}


#endif /* #if ! defined HAVE_NVJPEG */


/*
 * Local Variables:
 * mode: c
 * c-basic-offset:  4
 * End:
 */

