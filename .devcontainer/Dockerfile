FROM mcr.microsoft.com/devcontainers/base:ubuntu-24.04

USER root

ENV MPLBACKEND=Agg

RUN export DEBIAN_FRONTEND=noninteractive && \
  apt-get update && \
  apt-get install -y --no-install-recommends \
  build-essential cmake git pkg-config \
  libeigen3-dev nlohmann-json3-dev libgtest-dev \
  python3 python3-pip python3-venv python-is-python3 \
  openssh-client \
  && apt-get clean && rm -rf /var/lib/apt/lists/*

ENV VENV=/opt/venv
RUN python3 -m venv $VENV
ENV PATH="$VENV/bin:$PATH"

COPY scripts/requirements-plot.txt /tmp/requirements-plot.txt
RUN --mount=type=cache,target=/root/.cache/pip \
  pip install --upgrade pip && \
  pip install -r /tmp/requirements-plot.txt && \
  rm -f /tmp/requirements-plot.txt

USER vscode
