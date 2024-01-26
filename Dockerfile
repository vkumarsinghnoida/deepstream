FROM ubuntu:22.04

COPY shell.bash .
RUN chmod +x shell.bash && bash shell.bash
