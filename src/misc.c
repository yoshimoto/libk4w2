/**
 * @file   misc.c
 * @author Hiromasa YOSHIMOTO
 * @date   Wed May 13 18:31:12 2015
 * 
 * @brief  
 * 
 * 
 */

#include "module.h"

#include <stdlib.h> /* malloc(), free() */

#include <sys/stat.h> /* for open() */
#include <fcntl.h>
#include <unistd.h> /* for read() */

#include <string.h> /* for strerrno */
#include <errno.h> 

/** 
 * Allocates an array of #num elements of #size bytes each
 * and returns an array of pointers to the allocated memories.
 *
 * @param num 
 * @param size 
 * 
 * @return an array of num elements of pointers
 */
unsigned char **
allocate_bufs(int num, int size)
{
    int i;
    unsigned char ** buf;
    buf = (unsigned char**)malloc(num * sizeof(char*));
    if (!buf)
	goto err;
    buf[0] = (unsigned char*)malloc( size * num );
    if (!buf[0])
	goto err;
    for (i = 1; i < num; ++i) {
	buf[i] = buf[i-1] + size;
    }
    return buf;
err:
    return 0;
}

/** 
 * Releases an array allcated by allocate_bufs()
 * 
 * @param buf   a pointer to the array
 */
void
free_bufs(unsigned char **buf)
{
    if (buf) {
	free(buf[0]);
	free(buf);
    }
}

/** 
 * Loads file. This function searchs for a #filename in #searchpath, and
 * reads first one in the #searchpath.
 * 
 * @param searchpath 
 * @param num_searchpath 
 * @param filename 
 * @param buf 
 * @param bufsize 
 * 
 * @retval K4W2_SUCCESS  success
 */
int
k4w2_search_and_load(const char *searchpath[], size_t num_searchpath,
		     const char *filename, void *buf, size_t bufsize)
{
    size_t i;
    int r = K4W2_ERROR;
    for (i = 0; i<num_searchpath; ++i) {
	r = k4w2_load(searchpath[i], filename, buf, bufsize);
	if (K4W2_SUCCESS == r) {
	    VERBOSE("%s/%s was loaded successfully", searchpath[i], filename);
	    break;
	}
    }
    return r;
}

int
k4w2_load(const char *dirname, const char *filename, void *buf, size_t bufsize)
{
    int fd;
    ssize_t done = 0;
    char path[FILENAME_MAX];

    snprintf(path, sizeof(path), "%s/%s", dirname, filename);
    if (access(path, F_OK)) {
	/* no such file */
	return K4W2_ERROR;
    }

    fd = open(path, O_RDONLY);
    if (-1 == fd) {
	VERBOSE("open(%s) failed; %s", path, strerror(errno));
    } else {
	done = read(fd, buf, bufsize);
	if (-1 == done) {
	    VERBOSE("read(%s) failed; %s", path, strerror(errno));
	} else if ((size_t)done != bufsize) {
	    VERBOSE("load(%s) failed?", path);
	}
	close(fd);
    }
    return (done > 0 && (size_t)done == bufsize)?K4W2_SUCCESS:K4W2_ERROR;
}

int
k4w2_save(void *buf, size_t bufsize, const char *dirname, const char *filename)
{
    int fd;
    ssize_t done = 0;
    char path[FILENAME_MAX];

    snprintf(path, sizeof(path), "%s/%s", dirname, filename);

    fd = open(path, O_CREAT|O_WRONLY, 0644);
    if (-1 == fd) {
	VERBOSE("open(%s) failed; %s", path, strerror(errno));
    } else {
	done = write(fd, buf, bufsize);
	if (-1 == done) {
	    VERBOSE("write(%s) failed; %s", path, strerror(errno));
	} else if ((size_t)done != bufsize) {
	    VERBOSE("save(%s) failed?", path);
	}
	close(fd);
    }

    return (done > 0 && (size_t)done == bufsize)?K4W2_SUCCESS:K4W2_ERROR;
}


/** 
 * 
 * 
 * @param dirname 
 * @param color 
 * @param depth 
 * @param p0table 
 * 
 * @return 
 */
int
k4w2_camera_params_load(const char *dirname,
			struct kinect2_color_camera_param *color,
			struct kinect2_depth_camera_param *depth,
			struct kinect2_p0table *p0table)
{
    struct entry {
	const char *filename;
	void *ptr;
	size_t size;
    } e[] = {
	{"color.bin", color, sizeof(*color)},
	{"depth.bin", depth, sizeof(*depth)},
	{"p0table.bin", p0table, sizeof(*p0table)},
    };
    size_t i;
    for (i = 0; i<sizeof(e)/sizeof(*e); ++i) {
	int r;
	r = k4w2_load(dirname, e[i].filename, e[i].ptr, e[i].size);
	if (K4W2_SUCCESS != r) {
	    VERBOSE("failed to load(%s/%s)", dirname, e[i].filename);
	}
    }
    return K4W2_SUCCESS;
}

static int
k4w2_mkdir_p(const char *dirname)
{
    char path[FILENAME_MAX];
    char *cur;
    strncpy(path, dirname, sizeof(path));
    for (cur = path; *cur; ++cur) {
	if ('/'==*cur || '\0'==*(cur+1)) {
	    if ('/'==*cur)
		*cur = '\0';
	    if ( mkdir(path, 0744) ) {
		switch (errno) {
		case  EEXIST:
		    break;
		default:
		    VERBOSE("mkdir(%s) failed; %s", path, strerror(errno));
		    break;
		}
	    }
	    if ('\0'==*cur)
		*cur = '/';
	}
    }
    return K4W2_SUCCESS;
}

/** 
 * 
 * 
 * @param color 
 * @param depth 
 * @param p0table 
 * @param dirname 
 * 
 * @return 
 */
int
k4w2_camera_params_save(struct kinect2_color_camera_param *color,
			struct kinect2_depth_camera_param *depth,
			struct kinect2_p0table *p0table,
			const char *dirname)
{
    struct entry {
	const char *filename;
	void *ptr;
	size_t size;
    } e[] = {
	{"color.bin", color, sizeof(*color)},
	{"depth.bin", depth, sizeof(*depth)},
	{"p0table.bin", p0table, sizeof(*p0table)},
    };
    size_t i;
    int r;

    r = k4w2_mkdir_p(dirname);
    if (K4W2_SUCCESS != r) {
	goto exit;
    }

    r = K4W2_SUCCESS;
    for (i = 0; i<sizeof(e)/sizeof(*e); ++i) {
	if (K4W2_SUCCESS != k4w2_save(e[i].ptr, e[i].size, dirname, e[i].filename)) {
	    VERBOSE("failed to save(%s/%s)", dirname, e[i].filename);
	    r = K4W2_ERROR;
	}
    }
exit:
    return r;
}

/*
 * Local Variables:
 * mode: c
 * c-basic-offset:  4
 * End:
 */
