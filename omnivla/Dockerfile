FROM python:3.10-bookworm

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1
ENV PIP_NO_CACHE_DIR=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    git-lfs \
    build-essential \
    cmake \
    ninja-build \
    pkg-config \
    ffmpeg \
    libgl1 \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    && rm -rf /var/lib/apt/lists/*

RUN python -m pip install --upgrade pip setuptools wheel

# RTX 5090 / Blackwell-compatible PyTorch
RUN pip install \
    torch==2.7.1 \
    torchvision==0.22.1 \
    torchaudio==2.7.1 \
    --index-url https://download.pytorch.org/whl/cu128

WORKDIR /opt/OmniVLA

COPY src /opt/OmniVLA

# Patch upstream pyproject:
# - remove old Torch pins
# - fix missing comma after "av"
RUN sed -i \
    -e '/"torch==2.2.0",/d' \
    -e '/"torchvision==0.17.0",/d' \
    -e '/"torchaudio==2.2.0",/d' \
    -e 's/"av"$/"av",/' \
    pyproject.toml

# Validate pyproject before installing.
RUN pip install tomli && python - <<'PY'
import tomli

with open("pyproject.toml", "rb") as f:
    data = tomli.load(f)

deps = data["project"]["dependencies"]

assert not any(x.startswith("torch==2.2") for x in deps)
assert not any(x.startswith("torchvision==0.17") for x in deps)
assert not any(x.startswith("torchaudio==2.2") for x in deps)

print("pyproject.toml OK")
PY

# Install OmniVLA and upstream dependencies.
RUN pip install -e .

# Compatibility fixes discovered during working inference setup.
#
# huggingface_hub 2.x breaks transformers 4.40.1
# tensorflow-metadata newer releases break TF 2.15/protobuf compatibility
# newer wandb requires newer protobuf
RUN pip install --no-cache-dir --force-reinstall \
    "huggingface_hub==0.29.1" \
    "tensorflow-metadata==1.15.0" \
    "protobuf==3.20.3" \
    "wandb==0.17.0"

# Verify critical versions and GPU-capable Torch installation.
RUN python - <<'PY'
import importlib.metadata as m
import torch

expected = {
    "huggingface-hub": "0.29.1",
    "tensorflow-metadata": "1.15.0",
    "protobuf": "3.20.3",
    "wandb": "0.17.0",
}

print("PyTorch:", torch.__version__)

assert torch.__version__.startswith("2.7.1"), torch.__version__

for pkg, version in expected.items():
    actual = m.version(pkg)
    print(f"{pkg}: {actual}")
    assert actual == version, (pkg, actual, version)

print("OmniVLA environment OK")
PY

WORKDIR /opt/OmniVLA

CMD ["bash"]
