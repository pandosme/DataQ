PROG1	= DataQ
OBJS1	= main.c cJSON.c ACAP.c MQTT.c CERTS.c RadarDetection.c radarscene.pb-c.c protobuf-c.c
PROGS	= $(PROG1) 

PKGS = glib-2.0 gio-2.0 fcgi axevent libcurl

CFLAGS += $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH) pkg-config --cflags $(PKGS))
LDLIBS += $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH) pkg-config --libs $(PKGS))
LDLIBS  += -s -lm -ldl -lpthread
CFLAGS += -Wno-format-overflow

# libradar_scene_subscriber is loaded at runtime via dlopen() from the
# device's /usr/lib/ so there is no build-time link dependency.
# Headers are included directly from the project source tree.

all:	$(PROGS)

$(PROG1): $(OBJS1)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

clean:
	rm -rf $(PROGS) *.o $(LIBDIR) *.eap* *_LICENSE.txt manifest.json package.conf* param.conf

