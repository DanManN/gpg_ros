FROM osrf/ros:noetic-desktop-full

RUN \
  useradd user && \
  echo "user ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/user && \
  chmod 0440 /etc/sudoers.d/user && \
  mkdir -p /home/user && \
  chown user:user /home/user && \
  chsh -s /bin/bash user

RUN echo 'root:root' | chpasswd
RUN echo 'user:user' | chpasswd

# setup environment
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
RUN apt update && apt upgrade curl wget git -y

# add kitware repo to get latest cmake
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
RUN echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main' | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null
RUN curl -sSL https://apt.kitware.com/keys/kitware-archive-latest.asc | apt-key add -
RUN apt update

# install packages
RUN apt update && apt install -y --no-install-recommends \
    cmake python3-pip libeigen3-dev libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /home/user

USER user
CMD /bin/bash
SHELL ["/bin/bash", "-c"]

########################################
########### WORKSPACE BUILD ############
########################################
# Installing catkin package
RUN mkdir -p ~/gpg_ws/src
RUN cd ~/gpg_ws && \
    git clone https://github.com/DanManN/pygpg.git && \
    cd ~/gpg_ws/pygpg && \
    ./build_pygpg.sh
RUN pip install transformations
COPY --chown=user . /home/user/gpg_ws/src/gpg_ros
RUN source /opt/ros/noetic/setup.bash && \
    cd ~/gpg_ws && catkin_make

########################################
########### ENV VARIABLE STUFF #########
########################################
RUN echo "source ~/gpg_ws/devel/setup.bash" >> ~/.bashrc
#RUN echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc

CMD ["bash"]
