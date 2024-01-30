FROM ubuntu:22.04

COPY requirements.txt .
RUN	apt update \
	apt install sudo -y \
	sudo apt install openssh-server python3 python3-pip git curl wget nano x11vnc -y && sudo service ssh start\
	pip install -r requirements.txt \

RUN cd /tmp && \
    curl --location --remote-name https://github.com/novnc/noVNC/archive/refs/tags/v1.4.0.zip && \
    unzip v1.4.0.zip && \
    rm --force v1.4.0.zip && \
    cd noVNC-1.4.0/utils && \
    curl --location --remote-name https://github.com/novnc/websockify/archive/refs/heads/master.tar.gz && \
    tar xzf master.tar.gz && \
    mv websockify-master websockify && \
    rm --force master.tar.gz && \
    cd ../.. && \
    mv noVNC-1.4.0 /opt/noVNC

ENV DISPLAY=":0"
