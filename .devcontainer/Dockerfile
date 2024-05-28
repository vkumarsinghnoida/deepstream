FROM ros:humble-ros-base

RUN apt update && apt install git wget tar nano python3-pip python3 npm tmux neofetch -y
RUN wget https://github.com/neovim/neovim/releases/download/nightly/nvim-linux64.tar.gz && apt install tar && tar xzvf nvim-linux64.tar.gz
RUN git clone https://github.com/LazyVim/starter ~/.config/nvim
RUN curl -sSL https://raw.githubusercontent.com/aandrew-me/tgpt/main/install | bash -s /usr/local/bin
