PROG1	= DataQ
OBJS1	= main.c ACAP.c cJSON.c MQTT.c CERTS.c ObjectDetection.c TIME.c video_object_detection.pb-c.c protobuf-c.c
PROGS	= $(PROG1) 

PKGS = glib-2.0 gio-2.0 axevent axhttp video-object-detection-subscriber 

CFLAGS += $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH) pkg-config --cflags $(PKGS))
LDLIBS += $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH) pkg-config --libs $(PKGS))
LDLIBS  += -s -lm -ldl -lscene -laxparameter -lpthread
CFLAGS += -Wno-format-overflow

all:	$(PROGS)

$(PROG1): $(OBJS1)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

clean:
	rm -rf $(PROGS) *.o $(LIBDIR) *.eap* *_LICENSE.txt manifest.json package.conf* param.conf

