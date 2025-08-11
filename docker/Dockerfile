FROM debian:trixie-slim

ADD .cli-config.yml /usr/local/bin/.cli-config.yml

RUN apt-get update && apt-get install -y curl python3
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
RUN arduino-cli config init 
RUN arduino-cli core update-index
RUN arduino-cli core install esp8266:esp8266 --config-file /usr/local/bin/.cli-config.yml
WORKDIR /root/sketch


ADD helpers /usr/bin/