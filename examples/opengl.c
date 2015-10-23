#include <libk4w2/libk4w2.h>
#include <libk4w2/decoder.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <stdio.h>
#include <stdlib.h>

/* glfwPostEmptyEvent() is available in GLFW 3.1 or later */
#if (GLFW_VERSION_MAJOR > 3) || (GLFW_VERSION_MAJOR == 3 && GLFW_VERSION_MINOR >= 1)
#  define HAVE_GLFWPOSTEMPTYEVENT
#else
#  undef  HAVE_GLFWPOSTEMPTYEVENT
#endif


#define VERBOSE(fmt, ...) do { fprintf(stderr, __FILE__ ":%d " fmt "\n", __LINE__, ## __VA_ARGS__); } while(0)
#define ABORT(fmt, ...) do { VERBOSE(fmt, ## __VA_ARGS__ ); exit(EXIT_FAILURE); } while(0)
#define CHK( exp ) do { int res = exp; if (!res) { VERBOSE(#exp " failed."); } } while(0)

#define CHECK_GL() do {							\
	GLenum e;							\
	while ( (e = glGetError()) != GL_NO_ERROR ) {			\
	    VERBOSE("glGetError() returns '%s (0x%X)'",	glewGetErrorString(e), e ); \
	}								\
    } while(0)

static void
key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    switch (key) {
    case 'q':
    case 'Q':
    case GLFW_KEY_ESCAPE:
	  glfwSetWindowShouldClose(window, GL_TRUE);
	  break;
    }
}

GLFWwindow *
create_opengl_window(int w, int h, const char *windowname)
{
    glfwInit();

    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4); 
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    GLFWwindow* window = glfwCreateWindow(w, h, windowname, NULL, NULL);
    if (!window)
    {
	glfwTerminate();
	return NULL;
    }

    glfwSetKeyCallback(window, key_callback);

    glfwMakeContextCurrent(window);

    // Enable waiting for vsync.
    glfwSwapInterval(1);

    glewExperimental = GL_TRUE; // this will stop glew crashing on OSX :-/
    glewInit();

    // workaround for glewInit() issue.
    for (int retry = 0; retry<10; ++retry) {
	if (glGetError() == GL_NO_ERROR)
	    break;
    }

    VERBOSE("render:  %s", glGetString(GL_RENDERER));
    VERBOSE("version: %s", glGetString(GL_VERSION));
    return window;
}

enum {
    COLOR=0,
    DEPTH=1,
};
const void *last_ptr[2] = {0};
int last_len[2] = {0};

static void color_cb(const void *buffer, int length, void *userdata)
{
    if (length < 10000) {
	VERBOSE("bad color frame?");
	return;
    }

    last_ptr[COLOR] = buffer;
    last_len[COLOR] = length;

#if defined HAVE_GLFWPOSTEMPTYEVENT
    glfwPostEmptyEvent();
#endif
}

static void depth_cb(const void *buffer, int length, void *userdata)
{
    if (length != KINECT2_DEPTH_FRAME_SIZE*10) {
	VERBOSE("bad depth frame?");
	return;
    } 

    last_ptr[DEPTH] = buffer;
    last_len[DEPTH] = length;

#if defined HAVE_GLFWPOSTEMPTYEVENT
    glfwPostEmptyEvent();
#endif
}


static const char *glsl_source[2]={
    "#version 330\n"
    "in int gl_VertexID;"
    "out vec2 st;"
    "void main(void)"
    "{"
    "    vec2 p = vec2(gl_VertexID & 1, (gl_VertexID & 2)/2);"
    "    st = vec2(p.x, 1 - p.y);"
    "    gl_Position = vec4(p*2 - vec2(1), 0, 1);"
    "}",
    "#version 330\n"
    "out vec4 col;"
    "in vec2 st;"
    "uniform sampler2D texDepth;"
    "uniform sampler2D texColor;"
    "void main()"
    "{"
    "   if (st.x > 0.5) {"
    "       col = vec4(texture(texDepth, vec2((st.x-0.5)*2, st.y)).x / 5000. );"
    "   } else  {"
    "       col = texture(texColor, vec2(st.x*2,st.y));"
    "   }"
    "}"
};



static GLuint create_glsl_program()
{
    GLuint program = glCreateProgram();
    static const GLenum type[] = {GL_VERTEX_SHADER, GL_FRAGMENT_SHADER};

    for (size_t i=0; i<2; ++i) {
	GLuint shader = glCreateShader(type[i]);
	glShaderSource(shader, 1, &glsl_source[i], NULL);
	glCompileShader(shader);
	GLint compiled;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
	if (GL_TRUE != compiled) {
	    VERBOSE("compile error:: ");
	    GLsizei bufSize;
	    glGetShaderiv(shader, GL_INFO_LOG_LENGTH , &bufSize);
	    if (bufSize > 1) {
		GLsizei length;
		GLchar *infoLog = (GLchar*)malloc(bufSize);
		glGetShaderInfoLog(shader, bufSize, &length, infoLog);
		VERBOSE("%s", infoLog);
		free(infoLog);
	    }
	}
	glAttachShader(program, shader);
	glDeleteShader(shader);
	CHECK_GL();
    }
    glLinkProgram(program);
    CHECK_GL();

    return program;
}


int
main()
{
    k4w2_set_debug_level(1000);
    GLFWwindow *window = create_opengl_window(640*2,480, "opengl");

    k4w2_t ctx = k4w2_open(0, 0);
    if (!ctx) {
	ABORT("failed to open kinect device.");
    }

    k4w2_decoder_t decoder[2] = {0};
    /*
     * Note; If you want to use CUDA and OpenCL at the same time,
     * OpenCL context must be initialized first.  To guarantee this
     * initialization order, the code below initializes the depth
     * decoder first.
     */
    size_t num_slot = 1;
    unsigned int flags = K4W2_DECODER_ENABLE_OPENGL;
    CHECK_GL();
    decoder[DEPTH] = k4w2_decoder_open(K4W2_DECODER_DEPTH | flags , num_slot);
    CHECK_GL();
    decoder[COLOR] = k4w2_decoder_open(K4W2_DECODER_COLOR | flags , num_slot);
    CHECK_GL();

    {
	struct kinect2_color_camera_param colorparam;
	struct kinect2_depth_camera_param depthparam;
	struct kinect2_p0table p0table;
	CHK( K4W2_SUCCESS == k4w2_read_color_camera_param(ctx, &colorparam) );
	CHK( K4W2_SUCCESS == k4w2_read_depth_camera_param(ctx, &depthparam) );
	CHK( K4W2_SUCCESS == k4w2_read_p0table(ctx, &p0table) );

	CHK( K4W2_SUCCESS == k4w2_decoder_set_params(decoder[DEPTH],
						     &colorparam,
						     &depthparam,
						     &p0table) );
    }

    
    k4w2_set_color_callback(ctx, color_cb, decoder[COLOR]);
    k4w2_set_depth_callback(ctx, depth_cb, decoder[DEPTH]);


    CHECK_GL();
    GLuint texture[2];
    for (size_t i = 0; i < 2; ++i) {
	int slot = 0;
	CHK( K4W2_SUCCESS == k4w2_decoder_get_gl_texture(decoder[i], slot, 0, &texture[i]));
    }

    GLuint vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);
    CHECK_GL();

    GLuint prog = create_glsl_program();
    glUseProgram(prog);
    CHECK_GL();

    GLint loc;
    glActiveTexture(GL_TEXTURE0 + 2);
    glBindTexture(GL_TEXTURE_2D, texture[DEPTH]);
    loc = glGetUniformLocation(prog,"texDepth");
    if (loc>=0) {
	glUniform1i(loc, 2);
    }
    CHECK_GL();

    glActiveTexture(GL_TEXTURE0 + 3);
    glBindTexture(GL_TEXTURE_2D, texture[COLOR]);
    loc = glGetUniformLocation(prog,"texColor");
    if (loc>=0) {
	glUniform1i(loc, 3);
    }
    glActiveTexture(GL_TEXTURE0);
    CHECK_GL();

    CHK( K4W2_SUCCESS == k4w2_start(ctx) );
    glUseProgram(0);

    while (!glfwWindowShouldClose(window)) {
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	for (size_t i=0; i<2; ++i) {
	    if (last_ptr[i]) {
		int slot = 0;
		int res;
		res = k4w2_decoder_request(decoder[i], slot, last_ptr[i], last_len[i]);
		CHECK_GL();
		if (K4W2_SUCCESS != res) {
		    VERBOSE("k4w2_decoder_request() failed.");
		}
		last_ptr[i] = NULL;
	    }
	}

	glUseProgram(prog);
	CHECK_GL();
	glDrawArrays(GL_TRIANGLE_STRIP,0,4);
	CHECK_GL();
	glUseProgram(0);

	glfwSwapBuffers(window);

#if defined HAVE_GLFWPOSTEMPTYEVENT
	glfwWaitEvents();
#else
	glfwPollEvents();
#endif
    }


    k4w2_stop(ctx);
    for (size_t i = 0; i < 2; ++i) {
	k4w2_decoder_close(&decoder[i]);
    }
    k4w2_close(&ctx);

    glfwTerminate();
    return 0;
}
