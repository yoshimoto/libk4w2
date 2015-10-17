CPPFLAGS += `pkg-config libk4w2 --cflags`
LDFLAGS += `pkg-config libk4w2 --libs`

CPPFLAGS += `pkg-config glfw3 glew --cflags`
LDLIBS += `pkg-config glfw3 glew --libs`

TARGETS = simple opengl

all: $(TARGETS)
clean:
	$(RM) -f $(TARGETS) *.o *~
