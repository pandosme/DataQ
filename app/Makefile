PROG1	= DataQ
OBJS1	= main.c ACAP.c cJSON.c MQTT.c CERTS.c ObjectDetection.c VOD.c video_object_detection.pb-c.c protobuf-c.c  GeoSpace.c  Stitch.c\
        linmatrix/src/lm_log.c \
        linmatrix/src/lm_assert.c \
        linmatrix/src/lm_err.c \
        linmatrix/src/lm_mat.c \
        linmatrix/src/lm_chk.c \
        linmatrix/src/lm_shape.c \
        linmatrix/src/lm_permute.c \
        linmatrix/src/lm_oper.c \
        linmatrix/src/lm_oper_dot.c \
        linmatrix/src/lm_oper_norm.c \
        linmatrix/src/lm_oper_axpy.c \
        linmatrix/src/lm_oper_gemm.c \
        linmatrix/src/lm_lu.c \
        linmatrix/src/lm_qr.c \
        linmatrix/src/lm_symm_hess.c \
        linmatrix/src/lm_symm_eigen.c
PROGS	= $(PROG1) 

PKGS = glib-2.0 gio-2.0 fcgi axevent libcurl video-object-detection-subscriber

CFLAGS += $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH) pkg-config --cflags $(PKGS))
LDLIBS += $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH) pkg-config --libs $(PKGS))
LDLIBS  += -s -lm -ldl -lpthread
CFLAGS += -Ilinmatrix/inc
CFLAGS += -Wno-format-overflow

all:	$(PROGS)

$(PROG1): $(OBJS1)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

clean:
	rm -rf $(PROGS) *.o $(LIBDIR) *.eap* *_LICENSE.txt manifest.json package.conf* param.conf

