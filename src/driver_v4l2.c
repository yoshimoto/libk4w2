/**
 * @file   driver_v4l2.c
 * @author Hiromasa YOSHIMOTO
 * @date   Tue May 12 15:32:28 2015
 * 
 * @brief  video for linux 2 (v4l2) backend for libk4w2
 * 
 * This driver works only with gspca/kinect2 sensor driver on linux.
 */

#if ! defined WITH_V4L2
#  error "WITH_V4L2 is not defined, while driver_v4l2.c is complied"
#endif

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <poll.h>

#include <string.h> /* strerror() */
#include <linux/videodev2.h>

#include <stdlib.h> /* free() */

#include <assert.h>

#include "module.h"

typedef struct {
     void   *start;
     size_t  length;
} Buffer;

typedef struct {
     int              fd;
     Buffer          *buf;
     unsigned int     num_bufs;
} Camera;

static int open_camera(Camera *cam, const char *dev_name)
{
     struct stat st;

     if (-1 == stat(dev_name, &st)) {
	  VERBOSE("Cannot identify '%s'; %s", dev_name, strerror(errno));
	  return K4W2_ERROR;
     }

     if (!S_ISCHR(st.st_mode)) {
	  VERBOSE("%s is no device\n", dev_name);
	  return K4W2_ERROR;
     }

     cam->fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

     if (-1 == cam->fd) {
	  VERBOSE("Cannot open '%s': %d, %s\n",
		  dev_name, errno, strerror(errno));
	  return K4W2_ERROR;
     }
     return K4W2_SUCCESS;
}

static int xioctl(int fh, int request, void *arg)
{
    int r;

    do {
	r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno);

    return r;
}


static int read_frame(Camera *cam, k4w2_callback_t callback, void *userdata)
{
    struct v4l2_buffer buf;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(cam->fd, VIDIOC_DQBUF, &buf)) {
	switch (errno) {
	case EAGAIN:
	    return 0;

	case EIO:
	    /* Could ignore EIO, see spec. */

	    /* fall through */

	default:
	    ABORT("VIDIOC_DQBUF");
	}
    }

    assert(buf.index < cam->num_bufs);

    if (callback)
	callback(cam->buf[buf.index].start, buf.bytesused, userdata);

    if (-1 == xioctl(cam->fd, VIDIOC_QBUF, &buf))
	ABORT("VIDIOC_QBUF");

    return 1;
}

static void stop_camera(Camera *cam)
{
    enum v4l2_buf_type type;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(cam->fd, VIDIOC_STREAMOFF, &type))
	ABORT("VIDIOC_STREAMOFF");
}

static int start_camera(Camera *cam)
{
    unsigned int i;
    enum v4l2_buf_type type;

    for (i = 0; i < cam->num_bufs; ++i) {
	struct v4l2_buffer buf;

	CLEAR(buf);
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	buf.index = i;

	if (-1 == xioctl(cam->fd, VIDIOC_QBUF, &buf)) {
	    VERBOSE("VIDIOC_QBUF");
	    return K4W2_ERROR;
	}
    }
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(cam->fd, VIDIOC_STREAMON, &type)) {
	VERBOSE("VIDIOC_STREAMON");
	return K4W2_ERROR;
    }
    return K4W2_SUCCESS;
}

static void unmap_camera(Camera *cam)
{
    if (cam->buf) {
	unsigned int i;
	for (i = 0; i < cam->num_bufs; ++i) {
	    if (MAP_FAILED == cam->buf[i].start)
		continue;
	    if (-1 == munmap(cam->buf[i].start, cam->buf[i].length))
		ABORT("munmap");
	}
	free(cam->buf);
	cam->buf = NULL;
    }
}


static int mmap_camera(Camera *cam, int num_buf)
{
    unsigned int i;
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count = cam->num_bufs = num_buf;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(cam->fd, VIDIOC_REQBUFS, &req)) {
	VERBOSE("ioctl(VIDIOC_REQBUFS) failed");
	goto err;
    }

    if (req.count < 2) {
	VERBOSE("Insufficient buffer memory");
	goto err;
    }

    cam->buf = (Buffer*)calloc(req.count, sizeof(*cam->buf));

    if (!cam->buf) {
	VERBOSE("Out of memory");
	goto err;
    }

    for (i = 0; i < cam->num_bufs; ++i) {
	cam->buf[i].start = MAP_FAILED;
    }
     
    for (i = 0; i < cam->num_bufs; ++i) {
	struct v4l2_buffer buf;

	CLEAR(buf);

	buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory      = V4L2_MEMORY_MMAP;
	buf.index       = i;

	if (-1 == xioctl(cam->fd, VIDIOC_QUERYBUF, &buf)) {
	    VERBOSE("VIDIOC_QUERYBUF");
	    goto err;
	}


	//VERBOSE("buf.length: %d", buf.length);
	cam->buf[i].length = buf.length;
	cam->buf[i].start = mmap(NULL /* start anywhere */,
				  buf.length,
				  PROT_READ | PROT_WRITE /* required */,
				  MAP_SHARED /* recommended */,
				  cam->fd, buf.m.offset);

	if (MAP_FAILED == cam->buf[i].start) {
	    VERBOSE("mmap failed");
	    goto err;
	}
    }
    return K4W2_SUCCESS;

err:
    unmap_camera(cam);
    return K4W2_ERROR;
}

static void close_camera(Camera *cam)
{
    unmap_camera(cam);
    if (-1 == close(cam->fd))
	VERBOSE("close() failed");
    cam->fd = -1;
}

typedef struct {
     struct k4w2_driver_ctx k4w2; /* !! must be the first item */
     Camera cam[2];
     THREAD_T thread;
     volatile unsigned shutdown:1;
} k4w2_v4l2;

static int
k4w2_v4l2_open(k4w2_t ctx, unsigned int deviceid, unsigned int flags)
{
    k4w2_v4l2 * v4l2 = (k4w2_v4l2 *)ctx;
    CHANNEL ch;
    int res = K4W2_ERROR;

    for (ch = COLOR_CH; ch <= DEPTH_CH; ++ch) {
	v4l2->cam[ch].fd = -1;
    }

    for (ch = ctx->begin; ch <= ctx->end; ++ch) {

	char devfile[FILENAME_MAX];
	snprintf(devfile, sizeof(devfile), "/dev/video%d",
		 deviceid + ch);

	res = open_camera(&v4l2->cam[ch], devfile);
	if (K4W2_SUCCESS != res) {
	    VERBOSE("open_camera(%s) failed", devfile);
	    break;
	}

	res = mmap_camera(&v4l2->cam[ch], 8);
	if (K4W2_SUCCESS != res) {
	    VERBOSE("mmap_camera(%d) failed", ch);
	    break;
	}
    }

    return res;
}

static void *
k4w2_v4l2_thread_loop(void *arg)
{
    k4w2_v4l2 * v4l2 = (k4w2_v4l2*) arg;
    k4w2_t ctx = (k4w2_t) arg;
    const int num_fds = ctx->end - ctx->begin + 1;
    struct pollfd fds[2];
    CHANNEL ch;
    for (ch = ctx->begin; ch <= ctx->end; ++ch) {
	fds[ch].fd = v4l2->cam[ch].fd;
    }
    assert(num_fds == 2 || num_fds == 1);

    while (!v4l2->shutdown) {
	int r;
	for (ch = ctx->begin; ch <= ctx->end; ++ch) {
	    fds[ch].events = POLLIN;
	}

	r = poll(&fds[ctx->begin], num_fds, 1000);
	switch (r) {
	case -1:
	    VERBOSE("select failed");
	    break;
	case 0:
	    VERBOSE("select timeout");
	    break;
	}
	for (ch = ctx->begin; ch <= ctx->end; ++ch) {
	    if (fds[ch].revents)
		read_frame(&v4l2->cam[ch],
			   ctx->callback[ch],
			   ctx->userdata[ch]);
	}
    }
    return 0;
}

static int
k4w2_v4l2_start(k4w2_t ctx)
{
    k4w2_v4l2 * v4l2 = (k4w2_v4l2 *)ctx;
    CHANNEL ch;

    if (v4l2->thread)
	return K4W2_ERROR;

    v4l2->shutdown = 0;
    if (THREAD_CREATE(&v4l2->thread, k4w2_v4l2_thread_loop, ctx)) {
	VERBOSE("THREAD_CREATE() failed.");
	return K4W2_ERROR;
    }

    for (ch = ctx->begin; ch <= ctx->end; ++ch) {
	start_camera(&v4l2->cam[ch]);
    }
    return K4W2_SUCCESS;
}

static int
k4w2_v4l2_stop(k4w2_t ctx)
{
    k4w2_v4l2 * v4l2 = (k4w2_v4l2 *)ctx;
    CHANNEL ch;

    if (0== v4l2->thread)
	return K4W2_ERROR;

    v4l2->shutdown = 1;
    THREAD_JOIN(v4l2->thread);
    v4l2->thread = 0;

    for (ch = ctx->begin; ch <= ctx->end; ++ch) {
	stop_camera(&v4l2->cam[ch]);
    }
    return K4W2_SUCCESS;
}

static int
k4w2_v4l2_close(k4w2_t ctx)
{
    k4w2_v4l2 * v4l2 = (k4w2_v4l2 *)ctx;
    CHANNEL ch;

    k4w2_v4l2_stop(ctx);
    
    for (ch = ctx->begin; ch <= ctx->end; ++ch) {
	close_camera(&v4l2->cam[ch]);
    }
    return K4W2_SUCCESS;
}

static int
k4w2_v4l2_read_param(k4w2_t ctx, PARAM_ID id, void *param, int length)
{
    k4w2_v4l2 * v4l2 = (k4w2_v4l2 *)ctx;
    int r = K4W2_ERROR;
    static const struct cmd_tbl {
	unsigned long cmd;
	int len;
    } tbl[NUM_PARAMS] = {
	{VIDIOC_KINECT2_COLOR_PARAM, sizeof(struct kinect2_color_camera_param)},
	{VIDIOC_KINECT2_DEPTH_PARAM, sizeof(struct kinect2_depth_camera_param)},
	{VIDIOC_KINECT2_P0TABLE,     sizeof(struct kinect2_p0table)},
    };
    if (0 <= id && id < NUM_PARAMS) {
	if (tbl[id].len <= length) {
	    struct kinect2_ioctl_req  req;
	    req.len = tbl[id].len;
	    req.ptr = param;
	    r = ioctl(v4l2->cam[ctx->begin].fd, tbl[id].cmd, &req);
	    if (r) {
		VERBOSE("ioctl() failed; %s", strerror(errno));
		r = K4W2_ERROR;
	    }
	}
    }
    
    return r;
}

static const k4w2_driver_ops ops =
{
    .open	= k4w2_v4l2_open,
    .start	= k4w2_v4l2_start,
    .stop	= k4w2_v4l2_stop,
    .close	= k4w2_v4l2_close,
    .read_param = k4w2_v4l2_read_param,
};

REGISTER_MODULE(k4w2_driver_v4l2_init)
{
    k4w2_register_driver("v4l2", &ops, sizeof(k4w2_v4l2));
}
/*
 * Local Variables:
 * mode: c
 * c-basic-offset:  4
 * End:
 */
