ARG ARCH=armv7hf
ARG VERSION=12.8.0
ARG REPO=axisecp
ARG SDK=acap-native-sdk

FROM ${REPO}/${SDK}:${VERSION}-${ARCH}-ubuntu24.04
# Building the ACAP application
COPY ./app /opt/app/
WORKDIR /opt/app
RUN . /opt/axis/acapsdk/environment-setup* && acap-build . \
	-a 'settings/settings.json' \
	-a 'settings/events.json' \
	-a 'settings/subscriptions.json' \
	-a 'settings/mqtt.json'

