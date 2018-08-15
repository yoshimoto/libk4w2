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
    unsigned int texture_id;
    nvjpegJpegState_t jpeg;
    nvjpegImage_t image;
    cudaGraphicsResource_t gres;

    int phase;
} decoder_slot;

typedef struct {
    struct k4w2_decoder_ctx decoder; 

    nvjpegHandle_t handle;
    cudaStream_t   stream;
    decoder_slot *slot;

    nvjpegOutputFormat_t outputfmt;
    int nChannels;

    void *copy_buffer;
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
    } else {
	internalfmt = GL_RGB8;
	d->outputfmt = NVJPEG_OUTPUT_RGBI;
	d->nChannels = 3;
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

	cudaMalloc((void**)&slot->image.channel[0], 1920 * 1080 * d->nChannels);
	CUDA_CHECK_ERR();
	slot->image.pitch[0] = 1920 * d->nChannels;
    }

    if (type & K4W2_DECODER_ENABLE_OPENGL) {
	for (s = 0; s < ctx->num_slot; ++s) {
	    decoder_slot *slot = &d->slot[s];
	    glCreateTextures(GL_TEXTURE_2D, 1, &slot->texture_id);
	    glTextureStorage2D(slot->texture_id,
			       1, internalfmt, 1920, 1080);

	    // Prepares for opengl interoperation
	    if (1!=d->nChannels) {
		// CUDA cannot copy the image directory.
		d->copy_buffer = malloc(1920*1080*d->nChannels);
	    } else {
		cudaGraphicsGLRegisterImage(&slot->gres, slot->texture_id,
					    GL_TEXTURE_2D,
					    cudaGraphicsMapFlagsNone);
		CUDA_CHECK_ERR();
		d->copy_buffer = NULL;
	    }

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

    decode_phase3_if_needed(d, slot);

    if (slot->texture_id) {
	if (NULL == d->copy_buffer) {
	    // Update opengl's texture by using CUDA's opengl-interoperation feat.
	    cudaArray_t array;	
	    cudaGraphicsMapResources(1, &slot->gres, d->stream);
	    cudaGraphicsSubResourceGetMappedArray(&array, slot->gres, 0,0);	
	    cudaMemcpyToArray(array, 0, 0, slot->image.channel[0], 1920*1080*d->nChannels,cudaMemcpyDeviceToDevice);
	    cudaGraphicsUnmapResources(1, &slot->gres, d->stream);
	    CUDA_CHECK_ERR();
	} else {
	    // Update opengl's texture by using CPU.
	    //
	    // This is still required because opengl interoperation is not supported when the format of texture is GL_RGB8.
	    //
	    assert(d->copy_buffer);
	    cudaMemcpy(d->copy_buffer, slot->image.channel[0], 1920*1080*d->nChannels, cudaMemcpyDeviceToHost);
	    glTextureSubImage2D(slot->texture_id,
				0,
				0,0,
				1920, 1080,
				GL_RGB, GL_UNSIGNED_BYTE, d->copy_buffer);
	}
    }

    return K4W2_SUCCESS;
}



static int
color_nvjpeg_fetch(k4w2_decoder_t ctx, int slotNo, void *dst, int dst_length)
{
    decoder_nvjpeg * d = (decoder_nvjpeg *)ctx;
    decoder_slot *slot = &d->slot[slotNo % ctx->num_slot];

    cudaMemcpy(dst, slot->image.channel[0], dst_length, cudaMemcpyDeviceToHost);
    CUDA_CHECK_ERR();
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

	    if (slot->gres) {
		cudaGraphicsUnregisterResource(slot->gres);
		CUDA_CHECK_ERR();
	    }

	    if (slot->image.channel[0])
		cudaFree(slot->image.channel[0]);
	    if (slot->jpeg) {
		nvjpegJpegStateDestroy(slot->jpeg);
		slot->jpeg=0;
	    }
	}
	free(d->slot);
	d->slot = NULL;
    }
    if (d && d->copy_buffer) {
	free(d->copy_buffer);
	d->copy_buffer = NULL;
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


#endif /* #if ! defined HAVE_GPUJPEG */


/*
 * Local Variables:
 * mode: c
 * c-basic-offset:  4
 * End:
 */

