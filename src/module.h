/**
 * @file   module.h
 * @author Hiromasa YOSHIMOTO
 * @date   Tue May 12 18:30:29 2015
 * 
 * @brief  
 * 
 * 
 */

#ifndef __LIBK42W_PRIVATE_H_INCLUDED__
#define __LIBK42W_PRIVATE_H_INCLUDED__

#include "libk4w2/libk4w2.h"
#include "libk4w2/decoder.h"

#include <stdio.h>  /* fprintf() */
#include <string.h> /* memset() */

#ifdef __cplusplus
#  define EXTERN_C_BEGIN extern "C" {
#  define EXTERN_C_END   }
#else
#  define EXTERN_C_BEGIN
#  define EXTERN_C_END
#endif

EXTERN_C_BEGIN

/* === internal structure for kinect2 driver === */

typedef enum {
    COLOR_PARAM = 0,
    DEPTH_PARAM,
    P0TABLE,
    NUM_PARAMS,
} PARAM_ID;

/**
 * @struct k4w2_driver_ops
 */
typedef struct {
    int (*open)(k4w2_t ctx, unsigned int deviceid, unsigned int flags);
    int (*start)(k4w2_t ctx);
    int (*stop)(k4w2_t ctx);
    int (*close)(k4w2_t ctx);
    int (*read_param)(k4w2_t ctx, PARAM_ID id, void *param, int length);
} k4w2_driver_ops;

typedef enum {
    COLOR_CH = 0,
    DEPTH_CH = 1,
} CHANNEL;

/**
 * @struct k4w2_driver_ctx
 *
 */
struct k4w2_driver_ctx {
    /* 0: color, 1: depth */
    k4w2_callback_t callback[2]; 
    void *userdata[2];
    const k4w2_driver_ops *ops;

    CHANNEL begin; /* COLOR_CH or DEPTH_CH */
    CHANNEL end;   /* COLOR_CH or DEPTH_CH */
};

#define COLOR_ENABLED(ctx) (COLOR_CH == (ctx)->begin)
#define DEPTH_ENABLED(ctx) (DEPTH_CH == (ctx)->end)

/* === internal structure for kinect2 decoder === */

/**
 * @struct k4w2_decoder_ops
 *
 */
typedef struct {
    int (*open)(k4w2_decoder_t decoder, unsigned int type);
    int (*set_params)(k4w2_decoder_t decoder, 
		      struct kinect2_color_camera_param * color,
		      struct kinect2_depth_camera_param * depth,
		      struct kinect2_p0table * p0table);
    int (*request)(k4w2_decoder_t ctx, int slot, const void *src, int src_length);
    int (*wait)(k4w2_decoder_t ctx, int slot);
    int (*fetch)(k4w2_decoder_t ctx, int slot, void *dst, int dst_length);
    int (*close)(k4w2_decoder_t ctx);
} k4w2_decoder_ops;

/**
 * @struct k4w2_decoder_ctx  module.h
 */
struct k4w2_decoder_ctx {
    const k4w2_decoder_ops *ops;
    int num_slot;
};

/* ==== module management === */

//#define MODINIT(name)   __attribute__((constructor)) void name()
#define REGISTER_MODULE(name) void name()
#define INITIALIZE_MODULE(name)  do { void name(); name(); } while(0)

void k4w2_register_driver(const char *name,
			  const k4w2_driver_ops *ops,
			  int ctx_size);
void k4w2_register_decoder(const char *name,
			   const k4w2_decoder_ops *ops,
			   int ctx_size);

/* === thread === */
#include <pthread.h>
#define THREAD_T   pthread_t
#define THREAD_CREATE(th,func,arg)  pthread_create(th, NULL, func, arg)
#define THREAD_JOIN(th)             pthread_join(th, NULL)

#define MUTEX_T    pthread_mutex_t
#define MUTEX_INIT(mu)              pthread_mutex_init(mu, NULL)
#define MUTEX_INITIALIZER           PTHREAD_MUTEX_INITIALIZER
#define MUTEX_LOCK(mu)              pthread_mutex_lock(mu)
#define MUTEX_UNLOCK(mu)            pthread_mutex_unlock(mu)
#define MUTEX_DESTROY(mu)           pthread_mutex_destroy(mu)

#define COND_T     pthread_cond_t
#define COND_INIT(cond)             pthread_cond_init(cond, NULL)
#define COND_TIMEDWAIT(cond,mutex,abstime) pthread_cond_timedwait(cond, mutex, abstime)
#define COND_SIGNAL(cond)       pthread_cond_signal(cond)
#define COND_BROADCAST(cond)	pthread_cond_broadcast(cond)
#define COND_DESTROY(mu)	pthread_cond_destroy(mu)

/* === misc === */

extern int k4w2_debug_level;
#define OUTPUT(LV, fmt, ...) do { if (k4w2_debug_level >= (LV)) fprintf(stderr, __FILE__ ":%d " fmt "\n", __LINE__, ## __VA_ARGS__); } while(0)
#if _NDEBUG
#  define TRACE(fmt,...) (void)0
#else
#  define TRACE(fmt,...) OUTPUT(2, fmt, ## __VA_ARGS__)
#endif
#define VERBOSE(fmt,...) OUTPUT(1, fmt, ## __VA_ARGS__)
#define WARNING(fmt,...) OUTPUT(0, fmt, ## __VA_ARGS__)
#define ABORT(fmt,...) do { OUTPUT(0,fmt, ## __VA_ARGS__); exit(EXIT_FAILURE); } while (0)

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))
#define STR(x)  #x


unsigned char ** allocate_bufs(int num, int size);
void free_bufs(unsigned char **buf);

/* === file i/o === */
int k4w2_search_and_load(const char *searchpath[], size_t num_searchpath,
			 const char *filename,
			 void *buf, size_t bufsize);
int k4w2_load(const char *dirname, const char *filename,
	      void *buf, size_t bufsize);
int k4w2_save(void *buf, size_t size, const char *dirname,
	      const char *filename);


EXTERN_C_END

#endif /* #ifndef __LIBK42W_PRIVATE_H_INCLUDED__ */
/*
 * Local Variables:
 * mode: c
 * c-basic-offset:  4
 * End:
 */
