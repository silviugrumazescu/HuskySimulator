FROM nvcr.io/nvidia/isaac-sim:6.0.0

# A named account makes Kit resolve ~ correctly and preserves host ownership.
# Supplementary group 1234 can read NVIDIA's packaged app.
USER root
ARG USER_UID=1001
ARG USER_GID=1001
RUN set -eu; \
    if ! getent group "$USER_GID" >/dev/null; then groupadd --gid "$USER_GID" husky; fi; \
    account="$(getent passwd "$USER_UID" | cut -d: -f1)"; \
    if [ -n "$account" ]; then \
        usermod --home /isaac-sim --gid "$USER_GID" --append --groups 1234 "$account"; \
    else \
        useradd --uid "$USER_UID" --gid "$USER_GID" --groups 1234 --home-dir /isaac-sim --no-create-home husky; \
    fi
USER ${USER_UID}:${USER_GID}
