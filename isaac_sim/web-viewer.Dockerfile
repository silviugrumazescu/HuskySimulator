# NVIDIA's pinned WebRTC sample is generated inside the image, not in this repo.
FROM node:22-bookworm-slim AS build
WORKDIR /app
RUN npm config set @nvidia:registry https://edge.urm.nvidia.com/artifactory/api/npm/omniverse-client-npm/ && \
    npx --yes @nvidia/create-ov-web-rtc-app@2.1.5 --name=viewer --sample=local-sample --external --outputDir=/app sample-app --framework=react
WORKDIR /app/viewer
RUN sed -i "/signalingServer:/a\\    signalingPort: 49100,\n    mediaServer: new URLSearchParams(window.location.search).get('streamHost') || '127.0.0.1',\n    mediaPort: 47998,\n    forceWSS: false," src/main.tsx && \
    npm install --ignore-scripts && npm run build

FROM nginx:1.28-alpine
COPY --from=build /app/viewer/dist /usr/share/nginx/html
EXPOSE 80
