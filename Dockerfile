ARG ARCH=armv7hf
ARG VERSION=3.5
ARG UBUNTU_VERSION=20.04
ARG REPO=axisecp
ARG SDK=acap-sdk

#FROM ${REPO}/${SDK}:${VERSION}-${ARCH}-ubuntu${UBUNTU_VERSION}
FROM ${REPO}/${SDK}:${VERSION}-${ARCH}
# Building the ACAP application
COPY ./app /opt/app/
WORKDIR /opt/app
RUN . /opt/axis/acapsdk/environment-setup* && acap-build . \
	-a 'settings/settings.json' \
	-a 'settings/attributes.json' \
	-a 'settings/events.json' \
	-a 'settings/subscriptions.json' \
	-a 'settings/mqtt.json'

