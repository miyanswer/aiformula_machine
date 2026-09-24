#!/bin/bash
# Kvaser USB-CAN を can0 として 500kbps で起動する。
# コンテナ(docker/compose.jetson.yaml: network_mode host)内からもホストからも
# 何度実行しても失敗しないよう冪等にしてある:
#  - can0 が既に存在すれば modprobe は不要 (コンテナ内に /lib/modules が
#    無い場合でも止まらない)
#  - can0 が既に UP なら bitrate 変更は "Device or resource busy" になるので
#    触らない

CAN_IF="${CAN_IF:-can0}"
CAN_BITRATE="${CAN_BITRATE:-500000}"

if ! ip link show "${CAN_IF}" > /dev/null 2>&1; then
    sudo modprobe kvaser_usb || echo "[WARN] modprobe kvaser_usb failed (run it on the host if this is a container)."
    sleep 1
fi

if ! ip link show "${CAN_IF}" > /dev/null 2>&1; then
    echo "[ERROR] ${CAN_IF} not found. Is the Kvaser adapter connected? (container needs network_mode: host)" >&2
    exit 1
fi

if ip link show "${CAN_IF}" | grep -q "[<,]UP[,>]"; then
    echo "[INFO] ${CAN_IF} is already up."
else
    sudo ip link set "${CAN_IF}" type can bitrate "${CAN_BITRATE}"
    sudo ip link set "${CAN_IF}" up
    echo "[INFO] ${CAN_IF} up (bitrate ${CAN_BITRATE})."
fi
